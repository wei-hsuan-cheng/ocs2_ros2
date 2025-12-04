  - Stage cost (quadrupedal + arm)

    Define $\mathbf{r}_{\mathrm{IE}}(\mathbf{x}) = \boldsymbol{p}_{\mathrm{ee}}(\mathbf{x})$, $\mathbf{r}_{\mathrm{IE}}^{\mathrm{ref}}(t) = \boldsymbol{p}_{\mathrm{ref}}(t)$, and $\boldsymbol{\zeta}_{\mathrm{IE}}(\mathbf{x},t) = \mathbf{e}_o$ from above where $\{\mathrm{IE}\}$ is the relative pose of end-effector w.r.t. an inertial frame in space.
    
    Split the state into base+arm coordinates $\mathbf{x}_{\mathrm{r}}$ and (optional) object state $\mathbf{x}_{\mathrm{o}}$,
    and let $\mathbf{u}^{\mathrm{ref}}(t)$ be the nominal input (feedforward; zero for most demos).

    $$
    \begin{align*}
    \ell(\mathbf{x},\mathbf{u},t) = \
    & \alpha_1 \Big( \|\mathbf{r}_{\mathrm{IE}}(\mathbf{x}) - \mathbf{r}_{\mathrm{IE}}^{\mathrm{ref}}(t)\|^2_{\mathbf{Q}_{\mathrm{ee}_p}} \, + \|\boldsymbol{\zeta}_{\mathrm{IE}}(\mathbf{x},t)\|^2_{\mathbf{Q}_{\mathrm{ee}_o}} \Big) \quad + \\
    & \alpha_2 \|\mathbf{x}_{\mathrm{r}} - \mathbf{x}_{\mathrm{r}}^{\mathrm{ref}}(t)\|^2_{\mathbf{Q}_{\mathrm{r}}} \quad + \\
    & \alpha_3 \|\mathbf{x}_{\mathrm{o}} - \mathbf{x}_{\mathrm{o}}^{\mathrm{ref}}(t)\|^2_{\mathbf{Q}_{\mathrm{o}}} \quad + \\
    & \|\mathbf{u} - \mathbf{u}^{\mathrm{ref}}(t)\|^2_{\mathbf{R}} \quad + \\
    & \sum_i p\!\big(h_i(\mathbf{x})\big).
    \end{align*}
    $$

    Typical task selections

    - Free-motion base tracking: $\alpha_1 = 0$, $\alpha_2 = 1$, $\alpha_3 = 0$.
    - Free-motion end-effector tracking: $\alpha_1 = 1$, $\alpha_2 = 1$, $\alpha_3 = 0$.
    - Object manipulation: $\alpha_1 = 0$, $\alpha_2 = 1$, $\alpha_3 = 1$.