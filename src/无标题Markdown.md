# MPC算法推导

质点模型运动学方程

```math
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
v \cos{\theta} \\
v  \sin{\theta} \\
w
\end{bmatrix}
=
\begin{bmatrix}
\cos{\theta} & 0 \\
\sin{\theta} & 0 \\
0 & 1
\end{bmatrix}
\cdot
\begin{bmatrix}
v \\
w
\end{bmatrix}

```

令 `$f(X,U)=\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta}\ \end{bmatrix}X=\begin{bmatrix} x\\y\\\theta\end{bmatrix} U=\begin{bmatrix} v\\w\end{bmatrix}$` ,对`$f(X,U)$`在`$(X_r,U_r)$`做泰勒一阶展开得&#x20;

```math
f(X,U)= f(X_r,U_r)+ \frac{\partial f}{\partial X}|_{(X_r,U_r)} (X-X_r) + \frac{\partial f}{\partial U}|_{(X_r,U_r)} (U-U_r)
```

```math
即
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}\
\end{bmatrix}
-
\begin{bmatrix}
\dot{x_r} \\
\dot{y_r} \\
\dot{\theta_k}\
\end{bmatrix}
=
\begin{bmatrix}
0 & 0 &-v_r\sin{\theta_r} \\
0 & 0 &v_r\cos{\theta_r} \\
0 &0 &0
\end{bmatrix}
\begin{bmatrix}
x-x_r \\
y-y_r\\
\theta -\theta_r
\end{bmatrix}
+
\begin{bmatrix}
\cos{\theta_r} & 0\\
\sin{\theta_r} & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v-v_r \\
w-w_r\\
\end{bmatrix}
```

```math
而 \begin{bmatrix}
\dot{x_r} \\
\dot{y_r} \\
\dot{\theta_r}\
\end{bmatrix}
=
\begin{bmatrix}
v_r \cos{\theta_r} \\
v_r  \sin{\theta_r} \\
w_r
\end{bmatrix}
```

```math
得 \begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}\
\end{bmatrix}

=
\begin{bmatrix}
0 & 0 &-v_r\sin{\theta_r} \\
0 & 0 &v_r\cos{\theta_r} \\
0 &0 &0
\end{bmatrix}
\begin{bmatrix}
x-x_r \\
y-y_r\\
\theta -\theta_r
\end{bmatrix}
+
\begin{bmatrix}
\cos{\theta_r} & 0\\
\sin{\theta_r} & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v \\
w\\ 
\end{bmatrix}


```

```math
离散化得
\begin{bmatrix}
\frac{x_{k+1}-x_{k}}{T} \\
\frac{y_{k+1}-y_{k}}{T} \\
\frac{\theta_{k+1}-\theta_{k}}{T} \\
\end{bmatrix}

=
\begin{bmatrix}
0 & 0 &-v_r\sin{\theta_r} \\
0 & 0 &v_r\cos{\theta_r} \\
0 &0 &0
\end{bmatrix}
\begin{bmatrix}
x_k-x_r \\
y_k-y_r\\
\theta_k -\theta_r
\end{bmatrix}
+
\begin{bmatrix}
\cos{\theta_r} & 0\\
\sin{\theta_r} & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
v_k \\
w_k\\ 
\end{bmatrix}

```

```math
\begin{bmatrix}
x_{k+1}-x_r \\
y_{k+1}-y_r \\
\theta_{k+1}-\theta_r \\
\end{bmatrix}

=
\begin{bmatrix}
1 & 0 &-v_r\sin{\theta_r}T \\
0 & 1 &v_r\cos{\theta_r}T \\
0 &0 &1
\end{bmatrix}
\begin{bmatrix}
x_k-x_r \\
y_k-y_r\\
\theta_k -\theta_r
\end{bmatrix}
+
\begin{bmatrix}
\cos{\theta_r}T & 0\\
\sin{\theta_r}T & 0 \\
0 & T
\end{bmatrix}
\begin{bmatrix}
v_k \\
w_k\\ 
\end{bmatrix}
```

```math

\xi(k+1) = A_{k,t} \cdot \xi(k) + B_{k,t}\cdot u(k)  


```

```math
\\其中
\xi(k) = \begin{bmatrix}x_k-x_r \\ y_k-y_r \\\theta_k-\theta_r \end{bmatrix} ,
u(k)=\begin{bmatrix}v_k\\ w_k \end{bmatrix},
A_{k,t}=\begin{bmatrix}
1 & 0 &-v_r\sin{\theta_r}T \\
0 & 1 &v_r\cos{\theta_r}T \\
0 &0 &1
\end{bmatrix},
B_{k,t}=\begin{bmatrix}
\cos{\theta_r}T & 0\\
\sin{\theta_r}T & 0 \\
0 & T
\end{bmatrix}
```

目标函数设计

```math
u_0^* =\argmin_{\xi_k,u_k}{(\xi_N-\xi_{ref_N})^TQ(\xi_N-\xi_{ref_N})+\sum_{k=0}^{N-1}{ [   (\xi_k-\xi_{ref_k})^TQ(\xi_N-\xi_{ref_k}) +u_k^TRu_k}]}

```

```math
J={(\xi_N-\xi_{ref_N})^TQ(\xi_N-\xi_{ref_N})+\sum_{k=0}^{N-1}{ [   (\xi_k-\xi_{ref_k})^TQ(\xi_N-\xi_{ref_k}) +u_k^TRu_k}]}
```

```math
subject \quad to ： 
\left\{
\begin{aligned}
\xi_{k+1} = A\xi_k +B u_k \\
\xi_{min}\le\xi_k\le\xi_{max} \\
u_{min}\le u_k \le u_{max} \\
\end{aligned}
\right.



```

&#x20;

```math
\begin{array}{ll}
\operatorname{minimize} & \frac{1}{2} x^T P x+q^T x \\
\text { subject to } & l \leq A_c x \leq u
\end{array}
```

`$%$`

```math
P=\operatorname{diag}\left(Q, Q, \ldots, Q, R, \ldots, R\right)
```

```math
q=\left[\begin{array}{c}
-Q \xi_{ref_0} \\
-Q \xi_{ref_1}\\
\vdots \\
-Q \xi_{ref_N}\\
0 \\
\vdots \\
0
\end{array}\right]
```

```math
A_c=\left[\begin{array}{ccccc|cccc}
-I & 0 & 0 & \cdots & 0 & 0 & 0 & \cdots & 0 \\
A & -I & 0 & \cdots & 0 & B & 0 & \cdots & 0 \\
0 & A & -I & \cdots & 0 & 0 & B & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & -I & 0 & 0 & \cdots & B \\
\hline I & 0 & 0 & \cdots & 0 & 0 & 0 & \cdots & 0 \\
0 & I & 0 & \cdots & 0 & 0 & 0 & \cdots & 0 \\
0 & 0 & I & \cdots & 0 & 0 & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & I & 0 & 0 & \cdots & 0 \\
0 & 0 & 0 & \cdots & 0 & I & 0 & \cdots & 0 \\
0 & 0 & 0 & \cdots & 0 & 0 & I & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0 & 0 & 0 & \cdots & I
\end{array}\right]
```

```math
l=\left[\begin{array}{c}
- \xi_0 \\
0 \\
\vdots \\
0 \\
-\xi_{min} \\
\vdots \\
-\xi_{min} \\
-u_{min} \\
\vdots \\
-u_{min} \\
\end{array}\right]
u=\left[\begin{array}{c}
- \xi_0 \\
0 \\
\vdots \\
0 \\
-\xi_{max} \\
\vdots \\
-\xi_{max} \\
-u_{max} \\
\vdots \\
-u_{max} \\
\end{array}\right]
```

```math
x=\left[\begin{array}{c}
\xi_0\\
\xi_1 \\
\vdots \\
\xi_N \\
u_0 \\
u_1 \\
\vdots \\
u_{N-1} \\
\end{array}\right]
```

