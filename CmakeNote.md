cmake_minimum_required() : 指定使用的cmake最低版本

$ g++ *.cpp -std=c++11 -o app：g++指定编译器 GUN C++  *.cpp 表示当前路径下的所有 -std是在选c++的新特性
在文本中选中c++特性
set(CMAKE_CXX_STANDARD 11)


#cmake的输出路径
set(HOME /home/robin/Linux/Sort)
set(EXECUTABLE_OUTPUT_PATH ${HOME}/bin)


${PROJECT_SOURCE_DIR} 是cmake .. 指令  cmake的地址


aux_source_directory(< dir > < variable >) 搜索指定路径下的源文件

file(GLOB/GLOB_RECURSE 变量名 要搜索的文件路径和文件类型)
GLOB: 将指定目录下搜索到的满足条件的所有文件名生成一个列表，并将其存储到变量中。
GLOB_RECURSE：递归搜索指定目录，将搜索到的满足条件的文件名生成一个列表，并将其存储到变量中。

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) 设置动态库的输出路径
