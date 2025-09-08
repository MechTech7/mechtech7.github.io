---
layout: post
title:  "Bayesian Filtering"
date:   2025-09-06
math: True
categories: jekyll update
---


*The principles behind the Bayesian Filtering are helpful for undestanding filters such as the Kalman Filter and Particle Filters.  The Kalman Filter can actually be thought of as a Bayesian Filter that assumes that all state probablities follow a normal distribution.*


## Introduction
With the [Kalman Filter](https://mechtech7.github.io/jekyll/update/2025/09/04/kalman_filter_basics.html), we introduced the concept of a state-estimator that maintains a probability distribution defining the estimated state.  Within the Kalman filter, this distribution was a Gaussian Distribution with a known mean and covariance.  If we relax this assumption that our state estimate must be estimated by a normal distribution, we can let our state be estimated by other, more expressive, distributions (like the bimodal distribution).  

## Derivation
### Markov Assumption
The Markov Assumption assumes that all sufficient information to determine $x_{t+1}$ is stored in $x_t$.  This assumption means that we can write

$$
p(x|x_{1:t-1})=p(x|x_{t-1})
$$

Like the Kalman Filter, the Bayesian Filter has an Update Step and a Predict Step
### Update Step
When we recieve a new measurement $y_t$, we can use it to update our probability distribution for $x_t$ to find the distribution $p(x_t|y_{1:t})$.  

$$
\begin{matrix}
p(x_t | y_{1:t}) = \frac{p(y_{1:t} | x_t)p(x_t)}{p(y_{1:t})} \\
p(x_t | y_{1:t}) = \frac{p(y_t | x_t,y_{1:t-1})p(y_{1:t-1}|x_t)p(x_t)}{\int_{x_t}p(y_t|x_t)p(y_{1:t-1}|x_t)p(x_t)}\\
\end{matrix}
$$

Part of the above expression can be factored into the following

$$
\begin{align*}
p(x_t|y_{1:t-1})=\frac{p(y_{1:t-1}|x_t)p(x_t)}{\int_{x_t}p(y_{1:t-1}|x_t)p(x_t)} \\
p(x_t|y_{1:t-1})=\frac{p(y_{1:t-1}|x_t)p(x_t)}{p(y_{t-1})} \\
p(x_t|y_{1:t-1})p(y_{t-1})=p(y_{1:t-1}|x_t)p(x_t)
\end{align*}
$$

Substituting our new expression for $p(y_{1:t-1}\vert x_t)p(x_t)$ back into our original expression gives us

$$
\begin{matrix}
p(x_t | y_{1:t}) = \frac{p(y_t | x_t,y_{1:t-1})p(x_t|y_{1:t-1})p(y_{t-1})}{\int_{x_t}p(y_t|x_t)p(x_t|y_{1:t-1})p(y_{t-1})}\\\\
p(x_t | y_{1:t}) = \frac{p(y_t | x_t,y_{1:t-1})p(x_t|y_{1:t-1})p(y_{t-1})}{p(y_{t-1)}\int_{x_t}p(y_t|x_t)p(x_t|y_{1:t-1})}\\\\
p(x_t | y_{1:t}) = \frac{p(y_t | x_t,y_{1:t-1})p(x_t|y_{1:t-1})}{\int_{x_t}p(y_t|x_t)p(x_t|y_{1:t-1})}\\
\end{matrix}
$$

Our closed-form update step is then

$$
p(x_t | y_{1:t}) = \frac{p(y_t | x_t)p(x_t|y_{1:t-1})}{\int_{x_t}p(y_t|x_t)p(x_t|y_{1:t-1})}\\
$$

### Predict Step
At this stage, we would like to find the probability distribution for our next state, $p(x_{t+1}|y_t)$ given our probability distribution for $x_t$.

We can estimate this probability distribution by marginalizing over all possible $x_t$

$$
\begin{matrix}
p(x_{t+1}|y_t)=\int_{x_{t}}p(x_{t+1},x_{t}|y_t) \\
p(x_{t+1}|y_t)= \int_{x_t}p(x_{t+1}|x_t)p(x_t|y_t)
\end{matrix}
$$


>Within this derivation, we performed our marginalization step using integration. This assumes that our states $x_t$ were continuous.  For continuous states, we can replace the integral with a summation in all of our equations and the solutions will still hold.