import os

# 指定CPP文件所在的目录
cpp_directory = './'

# 指定输出的TXT文件
output_file = 'merge.txt'

with open(output_file, 'w') as f_out:
    # 遍历CPP文件所在的目录
    for filename in os.listdir(cpp_directory):
        # 只处理CPP文件
        if filename.endswith('.cpp'):
            # 写入文件名
            f_out.write('\n' + '-'*100 + '\n')
            f_out.write("下面这份代码文件是 " + filename + " 的内容,这份CPP文件不带.h文件\n")
            f_out.write('\n' + '-'*100 + '\n')
            # 打开CPP文件并读取内容
            with open(os.path.join(cpp_directory, filename), 'r') as f_in:
                f_out.write(f_in.read())
            
            # 写入分隔符
            f_out.write('\n' + '-'*100 + '\n')
