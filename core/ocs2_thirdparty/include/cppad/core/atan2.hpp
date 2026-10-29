# ifndef CPPAD_CORE_ATAN2_HPP
# define CPPAD_CORE_ATAN2_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
-------------------------------------------------------------------------------
$begin atan2$$
$spell
    Vec
    CppAD
    namespace
    std
    atan
    const
$$


$section AD Two Argument Inverse Tangent Function$$

$head Syntax$$
$icode%theta% = atan2(%y%, %x%)%$$


$head Purpose$$
Determines an angle $latex \theta \in [ - \pi , + \pi ]$$
such that
$latex \[
\begin{array}{rcl}
    \sin ( \theta )  & = & y / \sqrt{ x^2 + y^2 }  \\
    \cos ( \theta )  & = & x / \sqrt{ x^2 + y^2 }
\end{array}
\] $$

$head y$$
The argument $icode y$$ has one of the following prototypes
$codei%
    const AD<%Base%>               &%y%
    const VecAD<%Base%>::reference &%y%
%$$

$head x$$
The argument $icode x$$ has one of the following prototypes
$codei%
    const AD<%Base%>               &%x%
    const VecAD<%Base%>::reference &%x%
%$$

$head theta$$
The result $icode theta$$ has prototype
$codei%
    AD<%Base%> %theta%
%$$

$head Operation Sequence$$
The AD of $icode Base$$
operation sequence used to calculate $icode theta$$ is
$cref/independent/glossary/Operation/Independent/$$
of $icode x$$ and $icode y$$.

$head Example$$
$children%
    example/general/atan2.cpp
%$$
The file
$cref atan2.cpp$$
contains an example and test of this function.

$end
-------------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN CppAD namespace

inline float atan2(float x, float y)
{   return std::atan2(x, y); }

inline double atan2(double x, double y)
{   return std::atan2(x, y); }

// The code below is used as an example by the CondExp documentation.
// BEGIN CondExp
template <class Base>
AD<Base> atan2 (const AD<Base> &y, const AD<Base> &x)
{
    AD<Base> zero(0.);
    AD<Base> pi2(2. * atan(1.));
    AD<Base> pi(2. * pi2);

    // Backport upstream CppAD's axis-derivative fix: fabs has zero derivative at zero.
    // https://github.com/coin-or/CppAD/blob/master/include/cppad/core/atan2.hpp
    AD<Base> abs_x = CondExpGe(x, zero, x, -x);
    AD<Base> abs_y = CondExpGe(y, zero, y, -y);
    // Code generation may evaluate both expressions before selecting one.
    // Keep the unused denominator nonzero on the axes; the selected ratio is unchanged.
    AD<Base> one(1.);
    AD<Base> alpha = atan(abs_y / CondExpGt(abs_x, abs_y, abs_x, one));
    AD<Base> beta = pi2 - atan(abs_x / CondExpGt(abs_x, abs_y, one, abs_y));
    AD<Base> first = CondExpGt(abs_x, abs_y, alpha, beta);
    AD<Base> second = pi - first;
    AD<Base> third = -pi + first;
    AD<Base> fourth = -first;
    alpha = CondExpGe(y, zero, first, fourth);
    beta = CondExpGe(y, zero, second, third);
    return CondExpGe(x, zero, alpha, beta);
}
// END CondExp

template <class Base>
AD<Base> atan2 (const VecAD_reference<Base> &y, const AD<Base> &x)
{   return atan2( y.ADBase() , x ); }

template <class Base>
AD<Base> atan2 (const AD<Base> &y, const VecAD_reference<Base> &x)
{   return atan2( y , x.ADBase() ); }

template <class Base>
AD<Base> atan2
(const VecAD_reference<Base> &y, const VecAD_reference<Base> &x)
{   return atan2( y.ADBase() , x.ADBase() ); }

} // END CppAD namespace

# endif
