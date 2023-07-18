#!/usr/bin/env python3
# -*- coding=utf-8 -*-

import os
import logging
import subprocess
import zipfile
import shutil
import sys
import random
import string
import struct
import linecache

from tempfile import mkdtemp

def list_all_files(dir_path):
    files = []
    i = 0
    for filepath,dirnames,filenames in os.walk(dir_path):
        for filename in filenames:
            files.insert(i,os.path.join(filepath,filename))
            i = i + 1
    return files

def sign_package(package_path):
    i = 1
    # Unzip package file
    f = zipfile.ZipFile(package_path, 'r')
    for file in f.namelist():
        f.extract(file, tmp_dir)

    tmp_bin_file_path = os.path.join(tmp_check_dir,"binary.txt")
    tmp_bin_file = open(tmp_bin_file_path,'wb')

    if os.path.isfile(sign_file):
        os.remove(sign_file)

    for file in list_all_files(tmp_dir):
        # Start signing upgrade files
        cmd = 'cd %s && openssl dgst -sign %s -sha256 -hex %s >> %s' % (
            tmp_dir, rsa_key_file, file[len(tmp_dir)+1::], sign_file)
        logging.info(cmd)
        (status, output) = subprocess.getstatusoutput(cmd)
        if status:
            logging.error('run [%s] fail: %s' % (cmd, output))
            return None, output

        # will verify the signature by public key
        theline = linecache.getline(sign_file, i)
        linecache.clearcache()
        signature=theline.split(" ")
        rel_signature=signature[1]

        # tmp signature file , will be deleted
        tmp_bin_file = open(tmp_bin_file_path,'wb')
        for index in range(0,len(rel_signature)-1,2):
            val = rel_signature[index:index + 2]
            val_tmp = struct.pack('B',int(val,16))
            tmp_bin_file.write(val_tmp)

        tmp_bin_file.close()
        rsa_pubkey_file = os.path.join(os.path.dirname(rsa_key_file), 'pubkey.pem')
        # start verify signature
        cmd = 'openssl dgst -verify %s -sha256 -signature %s %s' % (
            rsa_pubkey_file, tmp_bin_file_path, file)
        logging.info(cmd)
        (status, output) = subprocess.getstatusoutput(cmd)
        if status:
            logging.error('run [%s] fail: %s' % (cmd, output))
            return None, output
        # clear signature content
        open(tmp_bin_file_path, 'wb').close()
        i = i + 1

    # Compress upgrade package
    cmd = 'cd %s && zip -r %s *' % (tmp_dir, dstfile)
    (status, output) = subprocess.getstatusoutput(cmd)
    if status:
        logging.error('run [%s] fail: %s' % (cmd, output))
        return None, output

    return dstfile, None

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage:%s input.zip output.zip rsa_priv.pem --use absolute path' % sys.argv[0])
        exit(-1)

    srcfile = sys.argv[1]
    dstfile = sys.argv[2]
    rsa_key_file = sys.argv[3]

    if not srcfile.endswith('.zip') or not os.path.isfile(srcfile):
        print('Error:%s not exist or format not .zip' % srcfile)
        exit(-1)

    if not os.path.isfile(rsa_key_file):
        print('Error:%s rsa_key_file not exist ')
        exit(-1)

    src_filename = os.path.basename(srcfile)
    src_filename = os.path.splitext(src_filename)[0]

    dst_filename = os.path.basename(dstfile)
    dst_filename = os.path.splitext(dst_filename)[0]

    # Check signature upgrade package naming
    if not dst_filename == src_filename + '_signed':
        print('Error:%s not equal %s_signed' % (dst_filename,src_filename))
        exit(-1)

    # Create temporary directory
    tmp_dir = mkdtemp()
    if not os.path.isdir(tmp_dir):
        print ('mkdtemp %s faild' % tmp_dir)
        exit(-1)

    tmp_check_dir = mkdtemp()
    if not os.path.isdir(tmp_check_dir):
        print ('mkdtemp %s faild' % tmp_check_dir)
        exit(-1)

    sign_file = os.path.join(tmp_dir, 'signature.txt')

    output, err = sign_package(srcfile)
    if not output:
        exit(-1)
    shutil.rmtree(tmp_dir)
    shutil.rmtree(tmp_check_dir)
