# **Control 4DOF Manipulator**
## **Point To Point Control**
### <u>**Analytical Solution**</u><br>
**Inverse Kinematics**<br>
![result](/src/figure/pick.gif)
## **Continuous Path Control**
### <u>**Analytical Solution**</u>
**The system is like :**<br>
![system](/src/figure/ana.png)<br><br>

**Inverse Kinematics ( [simple_ik.py](https://github.com/mitsumaru3291/arm_control/blob/master/src/analytical_solver/simple_ik.py "simple_ik") )**<br>
![result](/src/figure/ana_real.gif)

### <u>**Numerical Solution**</u>
**The system is like :**<br>
![system](/src/figure/num.png)<br><br>

**Newton-Raphson-Method ( [numerical_calc.py](https://github.com/mitsumaru3291/arm_control/blob/master/src/numerical_solver/numerical_calc.py "numerical_solver") )** <br>
![result](/src/figure/num_real.gif)<br>

**Levenberg-Marquardt-Method ( [LM_method.py](https://github.com/mitsumaru3291/arm_control/blob/master/src/numerical_solver/LM_method.py "LM_method") )** <br>
![result](/src/figure/LM.gif)

## **Reference**
* [ROSを使用したCRANE+の動かし方](https://www.rt-shop.jp/blog/archives/6419 "ROS CRANE")
* 細田 耕著，実践ロボット制御，オーム社，2019
* 杉原 知道著，Levenberg-Marquardt法による可逆性を問わない逆運動学，日本ロボット学会誌，Vol.29，No.3，pp.269∼277，2011
* 小川 鉱一ら著，初めて学ぶ基礎ロボット工学，東京電機大学出版局，1998 