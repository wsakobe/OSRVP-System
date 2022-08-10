# OSRVP-System
Oral Surgery Robot VIsual Positioning System

使用方法：

生成所需尺寸的HydraMarker后修改如下内容

-Data

dot_matrix.txt     -直接将HydraMarker编码存入即可，横纵坐标随意确定，但不可转置

value_matrix.txt  -将存入dot_matrix的编码输入linkTableGenerator.m中，得到此文件，无需更改

model_3D.txt      -将注册模型存入此文件，单位为mm
