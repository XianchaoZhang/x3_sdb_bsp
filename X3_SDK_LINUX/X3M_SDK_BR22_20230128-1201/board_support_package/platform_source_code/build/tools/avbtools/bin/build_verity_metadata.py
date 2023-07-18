#!/usr/bin/env python2
#
# Copyright (C) 2013 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import os
import sys
import struct
import shlex
import subprocess
import tempfile
import hashlib
import shutil

VERSION = 0
MAGIC_NUMBER = 0xb001b001
BLOCK_SIZE = 4096
METADATA_SIZE = BLOCK_SIZE * 8

class AvbError(Exception):
  """Application-specific errors.

  These errors represent issues for which a stack-trace should not be
  presented.

  Attributes:
    message: Error message.
  """

  def __init__(self, message):
    Exception.__init__(self, message)

class Algorithm(object):
  """Contains details about an algorithm.

  See the avb_vbmeta_image.h file for more details about algorithms.

  The constant |ALGORITHMS| is a dictionary from human-readable
  names (e.g 'SHA256_RSA2048') to instances of this class.

  Attributes:
    algorithm_type: Integer code corresponding to |AvbAlgorithmType|.
    hash_name: Empty or a name from |hashlib.algorithms|.
    hash_num_bytes: Number of bytes used to store the hash.
    signature_num_bytes: Number of bytes used to store the signature.
    public_key_num_bytes: Number of bytes used to store the public key.
    padding: Padding used for signature, if any.
  """

  def __init__(self, algorithm_type, hash_name, hash_num_bytes,
               signature_num_bytes, public_key_num_bytes, padding):
    self.algorithm_type = algorithm_type
    self.hash_name = hash_name
    self.hash_num_bytes = hash_num_bytes
    self.signature_num_bytes = signature_num_bytes
    self.public_key_num_bytes = public_key_num_bytes
    self.padding = padding

def encode_rsa_key(key_path):
  """Encodes a public RSA key in |AvbRSAPublicKeyHeader| format.

  This creates a |AvbRSAPublicKeyHeader| as well as the two large
  numbers (|key_num_bits| bits long) following it.

  Arguments:
    key_path: The path to a key file.

  Returns:
    A bytearray() with the |AvbRSAPublicKeyHeader|.
  """
  key = RSAPublicKey(key_path)
  if key.exponent != 65537:
    raise AvbError('Only RSA keys with exponent 65537 are supported.')
  ret = bytearray()
  # Calculate n0inv = -1/n[0] (mod 2^32)
  b = 2L**32
  n0inv = b - modinv(key.modulus, b)
  # Calculate rr = r^2 (mod N), where r = 2^(# of key bits)
  r = 2L**key.modulus.bit_length()
  rrmodn = r * r % key.modulus
  ret.extend(struct.pack('!II', key.num_bits, n0inv))
  ret.extend(encode_long(key.num_bits, key.modulus))
  ret.extend(encode_long(key.num_bits, rrmodn))
  return ret

# This must be kept in sync with the avb_crypto.h file.
#
# The PKC1-v1.5 padding is a blob of binary DER of ASN.1 and is
# obtained from section 5.2.2 of RFC 4880.
ALGORITHMS = {
    'NONE': Algorithm(
        algorithm_type=0,        # AVB_ALGORITHM_TYPE_NONE
        hash_name='',
        hash_num_bytes=0,
        signature_num_bytes=0,
        public_key_num_bytes=0,
        padding=[]),
    'SHA256_RSA2048': Algorithm(
        algorithm_type=1,        # AVB_ALGORITHM_TYPE_SHA256_RSA2048
        hash_name='sha256',
        hash_num_bytes=32,
        signature_num_bytes=256,
        public_key_num_bytes=8 + 2*2048/8,
        padding=[
            # PKCS1-v1_5 padding
            0x00, 0x01] + [0xff]*202 + [0x00] + [
                # ASN.1 header
                0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
                0x00, 0x04, 0x20,
            ]),
    'SHA256_RSA4096': Algorithm(
        algorithm_type=2,        # AVB_ALGORITHM_TYPE_SHA256_RSA4096
        hash_name='sha256',
        hash_num_bytes=32,
        signature_num_bytes=512,
        public_key_num_bytes=8 + 2*4096/8,
        padding=[
            # PKCS1-v1_5 padding
            0x00, 0x01] + [0xff]*458 + [0x00] + [
                # ASN.1 header
                0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
                0x00, 0x04, 0x20,
            ]),
    'SHA256_RSA8192': Algorithm(
        algorithm_type=3,        # AVB_ALGORITHM_TYPE_SHA256_RSA8192
        hash_name='sha256',
        hash_num_bytes=32,
        signature_num_bytes=1024,
        public_key_num_bytes=8 + 2*8192/8,
        padding=[
            # PKCS1-v1_5 padding
            0x00, 0x01] + [0xff]*970 + [0x00] + [
                # ASN.1 header
                0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
                0x00, 0x04, 0x20,
            ]),
    'SHA512_RSA2048': Algorithm(
        algorithm_type=4,        # AVB_ALGORITHM_TYPE_SHA512_RSA2048
        hash_name='sha512',
        hash_num_bytes=64,
        signature_num_bytes=256,
        public_key_num_bytes=8 + 2*2048/8,
        padding=[
            # PKCS1-v1_5 padding
            0x00, 0x01] + [0xff]*170 + [0x00] + [
                # ASN.1 header
                0x30, 0x51, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x03, 0x05,
                0x00, 0x04, 0x40
            ]),
    'SHA512_RSA4096': Algorithm(
        algorithm_type=5,        # AVB_ALGORITHM_TYPE_SHA512_RSA4096
        hash_name='sha512',
        hash_num_bytes=64,
        signature_num_bytes=512,
        public_key_num_bytes=8 + 2*4096/8,
        padding=[
            # PKCS1-v1_5 padding
            0x00, 0x01] + [0xff]*426 + [0x00] + [
                # ASN.1 header
                0x30, 0x51, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x03, 0x05,
                0x00, 0x04, 0x40
            ]),
    'SHA512_RSA8192': Algorithm(
        algorithm_type=6,        # AVB_ALGORITHM_TYPE_SHA512_RSA8192
        hash_name='sha512',
        hash_num_bytes=64,
        signature_num_bytes=1024,
        public_key_num_bytes=8 + 2*8192/8,
        padding=[
            # PKCS1-v1_5 padding
            0x00, 0x01] + [0xff]*938 + [0x00] + [
                # ASN.1 header
                0x30, 0x51, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86,
                0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x03, 0x05,
                0x00, 0x04, 0x40
            ]),
}

def run(cmd):
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    output, _ = p.communicate()
    print output
    if p.returncode:
        exit(-1)

def get_verity_metadata_size(data_size):
    return METADATA_SIZE

def build_metadata_block(verity_table, signature):
    table_len = len(verity_table)
    block = struct.pack("II256sI", MAGIC_NUMBER, VERSION, signature, table_len)
    block += verity_table
    block = block.ljust(METADATA_SIZE, '\x00')
    return block

def sign_verity_table(table, signer_path, key_path, signer_args=None):
    with tempfile.NamedTemporaryFile(suffix='.table') as table_file:
        with tempfile.NamedTemporaryFile(suffix='.sig') as signature_file:
            table_file.write(table)
            table_file.flush()
            if signer_args is None:
              cmd = [signer_path, table_file.name, key_path, signature_file.name]
            else:
              args_list = shlex.split(signer_args)
              cmd = [signer_path] + args_list + [table_file.name, key_path, signature_file.name]
            print cmd
            run(cmd)
            return signature_file.read()

def build_verity_table(block_device, data_blocks, root_hash, salt):
    table = "1 %s %s %s %s %s %s sha256 %s %s"
    table %= (  block_device,
                block_device,
                BLOCK_SIZE,
                BLOCK_SIZE,
                data_blocks,
                data_blocks,
                root_hash,
                salt)
    return table

def raw_sign(signing_helper, signing_helper_with_files,
             algorithm_name, signature_num_bytes, key_path,
             raw_data_to_sign):
  """Computes a raw RSA signature using |signing_helper| or openssl.

  Arguments:
    signing_helper: Program which signs a hash and returns the signature.
    signing_helper_with_files: Same as signing_helper but uses files instead.
    algorithm_name: The algorithm name as per the ALGORITHMS dict.
    signature_num_bytes: Number of bytes used to store the signature.
    key_path: Path to the private key file. Must be PEM format.
    raw_data_to_sign: Data to sign (bytearray or str expected).

  Returns:
    A bytearray containing the signature.

  Raises:
    Exception: If an error occurs.
  """
  p = None
  if signing_helper_with_files is not None:
    signing_file = tempfile.NamedTemporaryFile()
    signing_file.write(str(raw_data_to_sign))
    signing_file.flush()
    p = subprocess.Popen(
      [signing_helper_with_files, algorithm_name, key_path, signing_file.name])
    retcode = p.wait()
    if retcode != 0:
      raise AvbError('Error signing')
    signing_file.seek(0)
    signature = bytearray(signing_file.read())
  else:
    if signing_helper is not None:
      p = subprocess.Popen(
          [signing_helper, algorithm_name, key_path],
          stdin=subprocess.PIPE,
          stdout=subprocess.PIPE,
          stderr=subprocess.PIPE)
    else:
      # ['openssl', 'rsautl', '-sign', '-inkey', key_path, '-raw'],
      p = subprocess.Popen(
          ['openssl', 'rsautl', '-sign', '-inkey', key_path, '-raw'],
          stdin=subprocess.PIPE,
          stdout=subprocess.PIPE,
          stderr=subprocess.PIPE)
    (pout, perr) = p.communicate(str(raw_data_to_sign))
    retcode = p.wait()
    if retcode != 0:
      raise AvbError('Error signing: {}'.format(perr))
    signature = pout
    # signature = bytearray(pout)
    # signature_hex = binascii.b2a_hex(signature)
    # print ("signature: %s" %(signature))
    # print ("signature_hex_len: %d " % (len(signature_hex)))
    # print ("signature_hex: %s" %(signature_hex))
  if len(signature) != signature_num_bytes:
    raise AvbError('Error signing: Invalid length of signature')
  return signature

def build_verity_metadata(data_blocks, metadata_image, root_hash, salt,
        block_device, signer_path, signing_key, signer_args=None):

    algorithm_name = "SHA256_RSA2048"

    try:
      alg = ALGORITHMS[algorithm_name]
    except KeyError:
      raise AvbError('Unknown algorithm with name {}'.format(algorithm_name))

	# build the verity table
    verity_table = build_verity_table(block_device, data_blocks, root_hash, salt)

    verity_info = "out/verity_info"
    print ("%s" %(verity_table))
    with open(verity_info, "wb") as f:
        f.write(verity_table)

    # build the verity table signature
    binary_hash = bytearray()
    if algorithm_name != 'NONE':
      ha = hashlib.new(alg.hash_name)
      ha.update(verity_table)
      binary_hash.extend(ha.digest())

    padding_and_hash = str(bytearray(alg.padding)) + binary_hash
    signature = raw_sign(None, None, algorithm_name, alg.signature_num_bytes, signing_key, padding_and_hash)
    # signature = bytes.encode(signature_binary)

    # signature = sign_verity_table(verity_table, signer_path, signing_key, signer_args)
    # build the metadata block
    metadata_block = build_metadata_block(verity_table, signature)
    # write it to the outfile
    with open(metadata_image, "wb") as f:
        f.write(metadata_block)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    parser_size = subparsers.add_parser('size')
    parser_size.add_argument('partition_size', type=int, action='store', help='partition size')
    parser_size.set_defaults(dest='size')

    parser_build = subparsers.add_parser('build')
    parser_build.add_argument('blocks', type=int, help='data image blocks')
    parser_build.add_argument('metadata_image', action='store', help='metadata image')
    parser_build.add_argument('root_hash', action='store', help='root hash')
    parser_build.add_argument('salt', action='store', help='salt')
    parser_build.add_argument('block_device', action='store', help='block device')
    parser_build.add_argument('signer_path', action='store', help='verity signer path')
    parser_build.add_argument('signing_key', action='store', help='verity signing key')
    parser_build.add_argument('--signer_args', action='store', help='verity signer args')
    parser_build.set_defaults(dest='build')

    args = parser.parse_args()

    if args.dest == 'size':
        print get_verity_metadata_size(args.partition_size)
    else:
        build_verity_metadata(args.blocks / 4096, args.metadata_image,
                              args.root_hash, args.salt, args.block_device,
                              args.signer_path, args.signing_key,
                              args.signer_args)
