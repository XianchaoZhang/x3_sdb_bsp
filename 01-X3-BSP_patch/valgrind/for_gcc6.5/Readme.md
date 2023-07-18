valgrind_prerootfs.tar
下载到板子的/userdata/ 分区，解压出来
设置环境变量：
export PATH=/userdata/prerootfs/usr/bin:$PATH
export VALGRIND_LIB=/userdata/prerootfs/usr/lib/valgrind

执行：
valgrind --leak-check=full --show-reachable=yes --trace-children=yes  programs args

提供一个base版本：valgrind_minimum.zip
export VALGRIND_LIB=/userdata/valgrind/lib
其它与上面一致