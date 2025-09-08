---
layout: post
title:  "Kalman Filter Basics"
date:   2025-09-04
math: True
categories: jekyll update
---

*The Information about Kalman Filters here can be found in Chapter 1 of [Model Predictive Control: Theory, Computation, and Design](https://sites.engineering.ucsb.edu/~jbraw/mpc/MPC-book-2nd-edition-1st-printing.pdf)*
## System Description
We assume that the system whose state we are trying to estimate can be modeled as follows

$$
\begin{matrix}
x_{t+1}=Ax_t+Bu_t+Gw_t \\
y_t=Cx_t+Du_t+v_t \\
v\sim \mathcal{N}(0, R)\\
w\sim \mathcal{N}(0, Q)
\end{matrix}
$$
{: .mathjax-process}

This is a linear time-invariant system that is affected by noise from random variables $w\in\mathbb{R}^g$ and $v\in \mathbb{R}^p$.  The matrix $G\in\mathbb{R}^{n\times g}$ allows for additional tuning between the relationship between noise $w$ and system state.  


## Kalman Filter Derivation
### Linear Optimal Derivation
#### Normal Distribution Identities
Before we get into our derivation of the Kalman filter, we are going to introduce a couple of identities about normal distributions that will be useful for our derivation.

#### Joint Independent Normals
Given $x\sim\mathcal{N}(\mu_x, C_{x})$ and $y\sim\mathcal{N}(\mu_y, C_{y})$ are statistically independent normal rendom variables, the joint density is given as follows.  

$$
\begin{matrix}
p_{x,y}(x,y) = n(x, \mu_x, C_x)\cdot n(y, \mu_y, C_y) \\
\begin{bmatrix}
x \\
y
\end{bmatrix}
\sim \mathcal{N} \left(
\begin{bmatrix}
\mu_x \\
\mu_y
\end{bmatrix},
\begin{bmatrix}
C_x & 0 \\
0 & C_y
\end{bmatrix}
\right)
\end{matrix}
$$
{: .mathjax-process}

#### Linear Transformation of a Normal
Given $x\sim\mathcal{N}(\mu_x, C_{x})$  and the value $y=Ax$ is also a random variable sampled from the following distribution
$$
y\sim\mathcal{N}(A\mu_x, AC_xA^T)
$$
#### Conditional of a Joint Normal
Given jointly distributed $x,y$ the conditional density of $x\vert y$ is given as follows

$$
\begin{matrix}
p_{x|y}(x|y)=n(x, \mu_{x|y}, C_{x|y}) \\
\mu_{x|y}=\mu_{x|y}+C_{xy}C_{yy}^{-1}(y-\mu_y) \\
C_{x|y} = C_{xx}-C_{xy}C_{y}^{-1}C_{yx} \\
\end{matrix}
$$
{: .mathjax-process}

Our estimation of $\hat{x}_t$ starts at time $t=0$ with an initial guess for the mean and variance of our state's normal distribution.

$$
x_0\sim \mathcal{N}(\mu_{0}, Q_0)
$$
{: .mathjax-process}

#### Derivation

In practice, we often do not have access to information about $x_0$ so it is common to initialize $x_0$ to zero and $Q_0$ to some large matrix to represent our uncertainty about $x$.

$$
\begin{matrix}
\mu_0=0\\
Q_0=\alpha I, \textbf{ for } \alpha \gg 1
\end{matrix}
$$
{: .mathjax-process}

Given our initial estimate, we can further refine our estimate with measurement information $y_0$

$$
y_0=Cx_0+v_0
$$
{: .mathjax-process}

> Here we are neglecting control input $u$ for simplicity however, all results derived here can still be applied to systems with nonzero $u$ provided one subtracts $Bu$ and $Du$ from the $x$ and $y$ respectively.


Given $x_0$ and $y_0$, we would like to estimate the normal distribution of $x_{0}^{+}$ given $x_0$ and $y_0$. To do this, we can start by defining the relationship between $x_{0}^{+}, y_0, x_0, v_0$.

$$
\begin{bmatrix}
x_{0}^{+} \\
y_{0}
\end{bmatrix}
=
\begin{bmatrix}
I & 0 \\
C & I
\end{bmatrix}
\begin{bmatrix}
x_0 \\
v_0
\end{bmatrix}
$$
{: .mathjax-process}

The variable $\begin{bmatrix}x_0 \\ v_0 \end{bmatrix}$ is itself a random variable which comes from the following distribution

$$
\begin{bmatrix}x_{0}^{+} \\ v_0 \end{bmatrix} \sim \mathcal{N} \left(
\begin{bmatrix}
\mu_x \\
0
\end{bmatrix},
\begin{bmatrix}
Q_0 & 0 \\
0 & R
\end{bmatrix}
\right)
$$
{: .mathjax-process}

Using the definition from Linear Transformation of a Normal, we can define the joint distribution for $\begin{bmatrix}x_{0}^{+} \\ y_0 \end{bmatrix}$ as follows

$$
\begin{bmatrix}
x_{0}^{+} \\
y_0
\end{bmatrix}
\sim \mathcal{N} \left(
\begin{bmatrix}
\mu_0 \\
C\mu_0
\end{bmatrix},
\begin{bmatrix}
Q_0 & Q_0C^T \\
CQ_0 & CQ_0C^T + R
\end{bmatrix}
\right)
$$
{: .mathjax-process}

We would like to go from this joint distribution to the conditional distribution for  $x_{0}^{+}\vert y_0$ which we can do by applying the identity for the Conditional of a Joint Normal to get the following

$$
x_{0}^{+}|y_0\sim \mathcal{N}(\mu_0^{+}, Q_{0}^{+})\\
$$
{: .mathjax-process}

The definitions for $\mu_{0}^{+}$ and $Q_{0}^{+}$ are defined below.

$$
\begin{matrix}
\mu_{0}^{+}=\mu_{0}+L_0(y_0-C\mu_0) \\
L_0=Q_0C^T(CQ_0C^T+R)^{-1} \\
Q_{0}^{+}=Q_0-Q_0C^{T}(C^{T}Q_0c^{T}+R)^{-1}CQ_0
\end{matrix}
$$
{: .mathjax-process}

Once we have our refined distribution estimation for $x_{0}^{+}$, we can use it to make an estimate for $x_1$.  We can relate $x_1$ to $x_0$ as follows

$$
x_1=\begin{bmatrix}
A & G
\end{bmatrix}
\begin{bmatrix}
x_{0}^{+} \\
w_0
\end{bmatrix}
$$
{: .mathjax-process}

$x_1$ is a linear transformation of the normal variable $[x_0^+ w_0]^T$ meaning that we can use our Linear Transformation of a Normal identity to define the normal distribution for $x_1$ as follows

$$
x_1\sim \mathcal{N}\left(
A\mu_{0}^{+},
AQ_0^{+}A^T+GQG^T
\right)
$$
{: .mathjax-process}

Now our updated mean and variance estimate for our state $x_1$ are as follows

$$
\mu_1=A\mu_{0}^{+}, Q_1=AQ_0A^T+GQG^T
$$
{: .mathjax-process}

Using this $\mu_1$ and $Q_1$, we can restart this process as we did with $\mu_0$ and $Q_0$ and and estimate $\mu_2, Q_2, \mu_3, Q_3, \ldots$ 
### Kalman Filter Steps
Here we highlight the main steps of the Kalman Filter that can be repeated to estimate the normal distribution of $x_t$.

#### Initial estimate of $\mu_0, P_0$
Typically we have no prior information about $x_0$ so a common chosen initial distribution is a zero-mean normal distribution with a large covariance to indicate uncertainty as follows:

$$
\begin{matrix}
x_0\sim \mathcal{N}(\mu_0,P_0)\\
\mu_0=0\\
P_0=\alpha I\textbf{ for }\alpha\gg 0
\end{matrix}
$$


#### Calculation of $\mu_{k}^{+}, P_{k}^{+}$
Given a normal distribution estimate of $x_k\sim \mathcal{N}(\mu_k, P_k)$ we refine this estimated distribution by integrating measurement $y_k$.

The distribution is as follows

$$
x_{k}^{+}|y_{k}\sim \mathcal{N}(\mu_k^+, P_k^+)
$$
{: .mathjax-process}

Further definitions are below

$$
\begin{matrix}
\mu_{k}^{+}=\mu_{k}+L_k(y_k-C\mu_k) \\
L_k=P_kC^T(CP_kC^T+R)^{-1} \\
P_{k}^{+}=P_k-P_kC^{T}(C^{T}P_kC^{T}+R)^{-1}CP_k
\end{matrix}
$$
{: .mathjax-process}


Larger variance in $R$ corresponds to a lack of confidence in our sensors and leads to smaller update step size.  Smaller variance relates to greater confidence in sensors and a larger update step size.  Larger variance in $P$ corresponds to a lack of confidence in our state estimation and leads to larger update step size.  Smaller variance in $P$ corresponse to a smaller update in step size.  

#### Propagation of State forward to Estimate $\mu_{k+1}, P_{k+1}$
Using our refined normal distribution, we can step our estimate forward to $k+1$ to get the following distributions

$$
x_{k+1}\sim\mathcal{N}(\mu_{k+1}, P_{k+1})
$$

Further definitions are defined here

$$
\begin{matrix}
\mu_{k+1}=A\mu_{k}^{+} \\
P_{k+1}=AP_{k}A^{T}+GQG^T
\end{matrix}
$$





>The above formulation of the Kalman Filter has the control $u$ removed to slightly simplify the notation.  To write a Kalman Filter that takes into account controls requires us to modify the update of $\mu_t$ in our Update Step

>$$
\mu_k^{+}=\mu_k + L_k[y_k^{+}-(C\mu_k - Du_k)]
>$$

>As well as a modification in the belief propagation step

>$$
>\mu_{k+1}=A\mu_k^{+}+Bu_k
>$$



### Code Example
Below is a simple implementation of the Kalman Filter in Python for reference

```python
class KalmanFilter:
    def __init__(self, x_0, P_0, A, C, Q, R, G):
        # Set Current State
        self.curr_mu = x_0
        self.curr_P = P_0

        # Initialize State Transition and Probability Matrices
        self.A = A
        self.C = C
        self.Q = Q
        self.R = R
        self.G = G

    def integrate_observation(self, obs):
        cpr_mat = self.curr_P @ self.C.T @ 
			        np.linalg.inv(self.C @ 
			        self.curr_P @ self.C.T + self.R)
        L_k = cpr_mat

        self.curr_P = self.curr_P - cpr_mat @ self.C @ self.curr_P
        self.curr_mu = self.curr_mu + L_k @ (obs - self.C @ self.curr_mu)

    def step_state_forward(self):
        self.curr_mu = self.A @ self.curr_mu
        self.curr_P = self.A @ self.curr_P @ 
				        self.A.T + self.G @ self.Q @ self.G.T
    
    def get_state_estimate(self):
        return self.curr_mu
```

