---
jupytext:
  formats: ipynb,md:myst
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.16.3
kernelspec:
  display_name: Python 3 (ipykernel)
  language: python
  name: python3
---

```{code-cell} ipython3
:deletable: false
:editable: false

# Initialize Otter
import otter
grader = otter.Notebook(colab=True)
```

+++ {"id": "0vJLt3JRL9eR", "slideshow": {"slide_type": "notes"}}

# Python Tutorial With Jupyter notebooks

<a href="https://colab.research.google.com/github/wecacuee/ECE498-Mobile-Robots/blob/master/docs/notebooks/01-py-intro/Python_1.ipynb" >
<img src="imgs/colab-badge.svg" width = '' >
</a>

This tutorial was originally written by [Justin Johnson](https://web.eecs.umich.edu/~justincj/) for cs231n. It was adapted as a Jupyter notebook for Stanford cs228 by [Volodymyr Kuleshov](http://web.stanford.edu/~kuleshov/) and [Isaac Caswell](https://symsys.stanford.edu/viewing/symsysaffiliate/21335). This version has been adapted for Colab by Kevin Zakka for the Spring 2020 edition of [cs231n](https://cs231n.github.io/). It runs Python3 by default.

Yifeng Zhu made modifications for ECE491/591-Fall 2022.

Vikas Dhiman made modifications for ECE49x/59x

+++ {"id": "9t1gKp9PL9eV", "slideshow": {"slide_type": "slide"}}

## Introduction

Python is a great general-purpose programming language on its own, but with the help of a few popular libraries (numpy, scipy, matplotlib) it becomes a powerful environment for scientific computing.

![](imgs/xkcd-py-antigravity.png)

+++ {"id": "VHqICOIe3qym", "slideshow": {"slide_type": "skip"}}

## Python for C programmers

Python is simpler programming language as compared to C++. It has only a few [builtin](https://docs.python.org/3/library/functions.html#built-in-funcs) [concepts](https://docs.python.org/3/library/constants.html#built-in-consts) and a [large standard](https://docs.python.org/3/tutorial/stdlib.html) [library](https://docs.python.org/3/tutorial/stdlib2.html).

+++ {"slideshow": {"slide_type": "slide"}}

### Differences between C and Python

1. C is compiled vs Python is interpreted (Python executable converts each line to machine instructions one statement at a time.)
2. C is statically typed vs Python is dynamically typed.
3. C does not have memory management. You have to `free()` the memory yourself after you `malloc()` the memory. Python manages memory using reference counting.  When number of references to a memory location go to zero, the memory is freed.
4. C variable scope is defined by curly braces. Python variable scope is only defined by Classes, functions and modules.
5. C does not have operator overloading, which means that you cannot define the meaning of +, -, * for your own classes. Python can.

```{code-cell} ipython3
---
slideshow:
  slide_type: notes
---
# Print the philosophy of Python
import this
```

+++ {"id": "U1PvreR9L9eW", "slideshow": {"slide_type": "notes"}}

# In this tutorial, we will cover:

* Basic Python: Basic data types (Containers, Lists, Dictionaries, Sets, Tuples), Functions, Classes
* IPython: Creating notebooks, Typical workflows

+++ {"id": "nxvEkGXPM3Xh", "slideshow": {"slide_type": "notes"}}

## A Brief Note on Google Colab and Jupyter notebooks

[Google Colab](https://colab.research.google.com) is a Google hosted version of [Jupyter notebooks](https://docs.jupyter.org/en/latest/). Jupyter notebooks are great for teaching, presentations and trying out small portions of code, but they do not work well with multi-file projects. We enourage you to grow out of notebooks use the tool most appropriate for your need.

+++ {"id": "qLOhyq6wLRhn", "slideshow": {"slide_type": "notes"}}

## Jupyter notebooks and bash

Jupyter notebooks contain "cells". "Text cells" are rendered as markdown. "Code cells" are executed. All the stdout (standard output) and stderr (standard error) output is saved in the next cell called the "Output cell". The "Output cell" stores  the output untill it is explicity cleared or over written.

We will come to Python code in the next section. Let start with running bash commands in Jupyter notebooks.

Any command starting with a "!" is passed to the shell like bash.

```{code-cell} ipython3
---
id: CDId2qEZFIMI
slideshow:
  slide_type: skip
---
!echo $SHELL
```

+++ {"id": "nVjFkJLRFeSh", "slideshow": {"slide_type": "skip"}}

You can run any bash command and linux command that are provided by the OS that is running the Jupyter notebook.

```{code-cell} ipython3
---
id: 52LJZtRuFYau
slideshow:
  slide_type: skip
---
!uname -a
```

+++ {"id": "utWOD_pGGFjj", "slideshow": {"slide_type": "skip"}}

What is your username?

```{code-cell} ipython3
---
id: fxwhJKAlGE4d
slideshow:
  slide_type: skip
---
!id
```

+++ {"id": "TZFxrPAJGWuY", "slideshow": {"slide_type": "skip"}}

Recall the linux commands from ECE177: [one page quick reference on the linux command line](https://files.fosswire.com/2007/08/fwunixref.pdf)

```{code-cell} ipython3
---
id: 1L4Am0QATgOc
slideshow:
  slide_type: slide
---
!python3 --version
```

```{code-cell} ipython3
---
id: wQjDwAvy7OOs
slideshow:
  slide_type: skip
---
!ls
```

```{code-cell} ipython3
---
id: jxId3IU-9jcs
slideshow:
  slide_type: skip
---
!pwd
```

```{code-cell} ipython3
---
id: Ylc3H12l9mNg
slideshow:
  slide_type: slide
---
!mkdir test
```

```{code-cell} ipython3
# Let us not forget our real home location so that we can
# comeback to it
if not 'workbookDir' in globals(): # check if the variable exists
    import os
    workbookDir = os.getcwd() # save the CWD (current working directory) to a variable
```

```{code-cell} ipython3
# always start in that directory before the next few cells
%cd $workbookDir
```

+++ {"id": "1OxqT8lG-EKY", "slideshow": {"slide_type": "fragment"}}

You cannot use ! to navigate the file systems. In stead, you need to use % to change the directory.

```{code-cell} ipython3
!cd test
```

```{code-cell} ipython3
%pwd
```

```{code-cell} ipython3
---
id: V77Vq3jV9q_e
slideshow:
  slide_type: fragment
---
%cd test
```

```{code-cell} ipython3
---
id: HdMxLLrn9sd2
slideshow:
  slide_type: fragment
---
%pwd
```

+++ {"editable": true, "id": "mzsRQYp6X0C9", "nbgrader": {"cell_type": "code", "checksum": "3a6eb593a05595855b574976943d79c6", "grade": true, "grade_id": "cell-52a34a16a7edcbf3", "locked": false, "points": 2, "schema_version": 3, "solution": true, "task": false}, "slideshow": {"slide_type": "fragment"}}

The magic command `%alias` is used to define magic aliases to system commands. `%alias` without any arguments lists the predefined system commands mapped to magic commands.

```{code-cell} ipython3
%alias
```

Note that `%ls` is mapped to `ls -F -o --color`. You can get more help on the ls command on your system by running it with `--help` flag.

```{code-cell} ipython3
%ls --help
```

### Cell magics
Apart from [line magics](https://ipython.readthedocs.io/en/stable/interactive/magics.html#line-magics), Ipython has [cell magics](https://ipython.readthedocs.io/en/stable/interactive/magics.html#cell-magics) which start with double percentage sign like `%%`.

`%%capture` is one such cell magic that captures the output of a cell to a python variable.

The following example captures the output to a variable called capt.

```{code-cell} ipython3
%%capture capt
import sys
print('Hello stdout')
print('and stderr', file=sys.stderr)
```

Now you can inspect the `stdout` (Standard output) and `stderr` (Standard errorr) of the comands programatically. We will use this feature for outmatic grading of certain questions in this notebook.

```{code-cell} ipython3
capt.stdout, capt.stderr
```

```{code-cell} ipython3
capt.show()
```

```{code-cell} ipython3
%cd $workbookDir/test
!touch hi.txt
!sleep 1
!touch alive.txt
!sleep 1
!touch bye.txt
%cd $workbookDir
```

+++ {"deletable": false, "editable": false, "id": "ZjhFXmsTXzdU", "raw_mimetype": "", "slideshow": {"slide_type": "slide"}}

#### Question 1.
Go through the help of `%ls` command. Call `%ls` with the right flags to list files in the current directory sorted by time. It should list the files in the test directory in the following order `bye.txt  alive.txt  hi.txt` (2  marks)

```{code-cell} ipython3
%%capture lsoutput
%cd -q $workbookDir/test
# Write the magic %ls command with the correct flags here
...
%cd -q $workbookDir
```

```{code-cell} ipython3
lsoutput.show()
```

```{code-cell} ipython3
# The following test should pass
def test_ls_output(lsoutput):
    expected_output = 'bye.txt  alive.txt  hi.txt'
    assert lsoutput.stdout.strip() == expected_output.strip(), lsoutput.stdout
test_ls_output(lsoutput)
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q1")
```

+++ {"deletable": false, "editable": false, "id": "C1HnW5iYYxSl", "slideshow": {"slide_type": "slide"}}

<!-- BEGIN QUESTION -->

#### Question 2
What is the difference between "!cd directory" and "%cd
directory"?

+++

Describe your answer and reasoning here

+++ {"deletable": false, "editable": false}

<!-- END QUESTION -->


```{code-cell} ipython3
# always start in that directory before the next few cells
%cd $workbookDir
```

+++ {"id": "o75TUB7DG9Ya", "slideshow": {"slide_type": "skip"}}

## Running C programs in Jupyter notebooks

There are versions of Jupyter Notebooks where you can run C programs line by line. For example, [xeus-cling project](https://mybinder.org/v2/gh/jupyter-xeus/xeus-cling/stable?filepath=notebooks/xcpp.ipynb). Google does not support that however, so we are going to use the [magic command](https://ipython.readthedocs.io/en/stable/interactive/magics.html) : `%%witefile filename` to write C program to file which can then be compiled and executed.

```{code-cell} ipython3
---
id: 2aQSObpVG8hU
slideshow:
  slide_type: skip
---
%%writefile helloworld.c
#include <stdio.h>
int main() {
  printf("hello world from C!\n");
}
```

+++ {"id": "cdHr6_h7J4R_", "slideshow": {"slide_type": "skip"}}

When you start a "cell" with magic command `%%writefile <filename>`, the remaining contents of the cell get written into a the file with given filename. You can then manipulate the file like you would have in a terminal.

```{code-cell} ipython3
---
id: n-7uUzPwILGL
slideshow:
  slide_type: skip
---
!gcc helloworld.c -o helloworld && ./helloworld
```

+++ {"id": "JAFKYgrpL9eY", "slideshow": {"slide_type": "skip"}}

## Basics of Python

+++ {"id": "RbFS6tdgL9ea", "slideshow": {"slide_type": "skip"}}

Python is a high-level, dynamically typed programming language. Python code is often said to be almost like pseudocode, since it allows you to express very powerful ideas in very few lines of code while being very readable. As an example, here is an implementation of the classic quicksort algorithm in Python:

```{code-cell} ipython3
---
id: cYb0pjh1L9eb
slideshow:
  slide_type: skip
---
def quicksort(arr):
    if len(arr) <= 1:
        return arr
    pivot = arr[len(arr) // 2] # Split arr in half
    left = [x for x in arr if x < pivot] # Take elements smaller than pivot
    middle = [x for x in arr if x == pivot]
    right = [x for x in arr if x > pivot] # Take elements greater than pivot
    return quicksort(left) + middle + quicksort(right) # Concatenate lists

print(quicksort([3,6,8,10,1,2,1]))
```

+++ {"id": "XidR_BLv7cDc", "slideshow": {"slide_type": "slide"}}

#### Hello world in Python
Print the hello world! Unlike C, Python does not "need" a main function to run. All the lines are executed one by one. Semicolons are optional.

```{code-cell} ipython3
---
id: tjdWIXe-7U5N
slideshow:
  slide_type: fragment
---
print("hello world from Python!")
```

+++ {"id": "b3_90a8xKaH5", "slideshow": {"slide_type": "fragment"}}

You can run this single line from a python file.

```{code-cell} ipython3
---
id: 5Yqjvb3UKPmK
slideshow:
  slide_type: fragment
---
%%writefile helloworld.py
print("hello world from Python file!")
```

+++ {"id": "G3ynPyM3Kw1v", "slideshow": {"slide_type": "fragment"}}

You can run any python file using the python executable.

```{code-cell} ipython3
---
id: num8i6kYKjWY
slideshow:
  slide_type: fragment
---
!python3 helloworld.py
```

Although all lines are executed, it is customary to separate function and class definitions from what can be executed using `if __name__ == '__main__'` condition. This class is only true if the file is called as a top-level script.

```{code-cell} ipython3
%%writefile helloworld_main.py
def foo():
    print("Function foo was called")

print("Inside the script ", __name__)
if __name__ == '__main__':
    print("Hello world from __name___ == '__main__' condition")
```

```{code-cell} ipython3
!python3 helloworld_main.py
```

Having the `__name__ == '__main__'` condition allows one to `import helloworld-main`, without executing certain lines of code.

```{code-cell} ipython3
import helloworld_main
```

Note that with `import` statement the module name `helloworld_name` is assigned to the special variable `__name__`. Learn more [here](https://docs.python.org/3/library/__main__.html).

+++ {"id": "2FSfcsAeRkJm", "slideshow": {"slide_type": "slide"}}

Jupyter cells automatically print the last expression to stdout. Specifically, the last statement must be an [expression statement](https://docs.python.org/3/reference/simple_stmts.html#expression-statements).

For example, you do not need the `print()` function to print "hello world!" from Jupyter cell.

```{code-cell} ipython3
---
id: zURjvDyaSchs
slideshow:
  slide_type: fragment
---
"Hello world from Jupyter cell!"
```

+++ {"id": "UzEUx4hUShhj", "slideshow": {"slide_type": "slide"}}

You can suppress the output by making the last statement, not an "expression statement". You can do it by assining this to another variable, for example.

```{code-cell} ipython3
---
id: iopRBYOZSvON
slideshow:
  slide_type: fragment
---
x = "Hello world from Jupyter cell!" # this will not be printed by Juputer cell
```

+++ {"id": "Jy_cMvLyS7Ac", "slideshow": {"slide_type": "skip"}}

Just listing the variables in the last line of the Code cell, prints them to the output.

```{code-cell} ipython3
---
id: INYoz0ixS5Xy
slideshow:
  slide_type: skip
---
x = "Hello world from Jupyter cell!"
x # This will be printed
```

+++ {"id": "dDjXCIkfTRWP", "slideshow": {"slide_type": "fragment"}}

Just like Matlab, you can suppress the output of a variable by using semicolon.

```{code-cell} ipython3
---
id: -f2VaTRxTK8Q
slideshow:
  slide_type: fragment
---
x = "Hello world from Jupyter cell!"
x; # This will NOT be printed
```

+++ {"id": "ZebqXfrjTdWf", "slideshow": {"slide_type": "skip"}}

Remember, this printing without `print()` function is a feature of Jupyter notebooks, not Python language.

+++ {"id": "q25CEAWI5Ue6", "slideshow": {"slide_type": "slide"}}

### Builin data types

The principal built-in types are

1. Numerics
2. Sequences
3. Mappings
4. Classes
5. Instances
6. Exceptions.

+++ {"slideshow": {"slide_type": "slide"}}

#### Numerics
* **int** (integers): numbers without a decimal ( Integers have unlimited precision. )
* **float** (floating point numbers): numbers with a decimal (  Floating point numbers are usually implemented using double in C; information about the precision and internal representation of floating point numbers for the machine on which your program is running is available in `sys.float_info`. )
* **bool** (booleans): True or False values

+++ {"slideshow": {"slide_type": "slide"}}

#### Sequences
* **str** (strings): usually to represent text. However, anything that is wrapped in quotes (single or double) is treated as a string.
* **list** (lists): Like a C array, more like C++ std::vector. Dynamically allocated and memory managed.
* **tuple** : Like lists but immutable.
* **set**: Like lists, mutable. Checks for uniqueness of each value.

+++ {"slideshow": {"slide_type": "slide"}}

#### Mappings

* **dict** (mappings): Hash tables for key-value pairs. The keys need to be hashable.

```{code-cell} ipython3
---
id: sIl43SzQVlGr
slideshow:
  slide_type: skip
---
import sys
sys.float_info # Prints the system's default float size
```

+++ {"id": "DL5sMSZ9L9eq", "slideshow": {"slide_type": "slide"}}

#### Numbers

+++ {"id": "MGS0XEWoL9er", "slideshow": {"slide_type": "fragment"}}

Integers and floats work as you would expect from other languages:

```{code-cell} ipython3
---
id: KheDr_zDL9es
slideshow:
  slide_type: fragment
---
x = 3
(x, type(x))
```

```{code-cell} ipython3
---
id: sk_8DFcuL9ey
slideshow:
  slide_type: fragment
---
print(x + 1)   # Addition
print(x - 1)   # Subtraction
print(x * 2)   # Multiplication
print(x ** 2)  # Exponentiation (not available in C)
```

```{code-cell} ipython3
---
id: U4Jl8K0tL9e4
slideshow:
  slide_type: fragment
---
x += 1    # x = x + 1
print(x)
x *= 2    # x = x * 2
print(x)
```

```{code-cell} ipython3
---
id: w-nZ0Sg_L9e9
slideshow:
  slide_type: skip
---
y = 2.5
print(type(y))
print(y, y + 1, y * 2, y ** 2)
```

#### IEEE 754 Double-precision floating-point format

![](imgs/IEEE_754_Double_Floating_Point_Format.png)

Every floating point number in modern computers is stored in IEEE 754 format. It has three parts, sign $s$ ($\pm 1$), exponent $e$ and fraction $f$. Here's some code to find the exponent part and fraction part of a fraction. The sign is +1 for positive numbers and -1 for negative numbers. The fraction $f$ is always between 0 and 1. The exponent part is responsible for scaling the number so that 1+f is between 1 and 2. The value of the number is $s \times (1+f) \times 2^e$.

+++

The following function converts a number to its binary representation by repeated division. Play with it convert some numbers whose binary representation you know. Maybe try to write an inverse function that converts from binary representation to its value, by repeated multiplication.

```{code-cell} ipython3
import math
def value2binary(n, base=2):
    """
    Find binary representation of n by repeated division
    """
    bits = []
    ncopy = n
    while ncopy > 0:
        ncopy, remainder = divmod(ncopy, base)
        bits.append(remainder)
    return list(reversed(bits))
```

```{code-cell} ipython3
value2binary(1024)
```

The following function finds the fraction $(1+f)$ and the exponent part $e$ so that $n = f \times 2^{e}$. This does not work for 0, so we dont even try.

```{code-cell} ipython3
def ieee754_exponent(n, expbits=11):
    """
    Find the exponent of n in ieee 754 representation.

    It returns both frac and exp, so that the original number is
    frac x 2^{-exp}
    """
    exp = 0
    if n == 0:
        return (None, 0)
    frac = n
    # Try for maximum number of expbits
    # The basic logic is that if the number is smaller than 1
    # then keep multiplying it by 2 until it becomes bigger than 1
    # and keep a track of how many times we multiplied it by 2
    # as the exponent $e$.
    # Do the same if the number is bigger than 2, but divide it by 2
    for _ in range(expbits-1):
        if 1 <= abs(frac) and abs(frac) < 2:
            break
        elif abs(frac) > 2:
            frac /= 2
            exp += 1
        else: # frac <= 1
            frac *= 2
            exp -= 1
    if abs(frac) < 1: raise ValueError("Number too small")
    if abs(frac) >= 2: raise ValueError("Number too big")
    return exp, frac
```

Try the above function for different numbers and see if it gives reasonable answers.

```{code-cell} ipython3
ieee754_exponent(0.1)
```

Check if $1.6 \times 2^{-4}$ is indeed 0.1

```{code-cell} ipython3
1.6*(2**(-4))
```

```{code-cell} ipython3
ieee754_exponent(0.1), ieee754_exponent(0.2), ieee754_exponent(0.3)
```

```{code-cell} ipython3
ieee754_exponent(0.125), ieee754_exponent(0.25), ieee754_exponent(0.375)
```

We can write an inverse function to create an automatic test case. This is called unit testing. If our function is wrong, this automatic test case should stop us from going forward without fixing it.

```{code-cell} ipython3
import random
def exponent_frac_to_number(exp, frac):
    return frac * 2**exp

def test_ieee754_exponent():
    for _ in range(100): # do this hundred times
        n = random.random()*200-100 # pick a random number between -100 and 100
        exp, frac = ieee754_exponent(n) # Conversion
        nagain = exponent_frac_to_number(exp, frac) # we should get n back again
        assert abs(nagain - n) < 2**42, f"test failed for {n}, {exp}, {frac}"

test_ieee754_exponent()
```

#### Imprecision in representing floating point numbers

Consider the following test. Adding 0.2 and 0.1 and comparing it to 0.3 fails. Why?

```{code-cell} ipython3
---
id: zx8-hIHea3FP
slideshow:
  slide_type: slide
---
x = 0.2 + 0.1
x == 0.3
```

```{code-cell} ipython3
---
id: zx8-hIHea3FP
slideshow:
  slide_type: slide
---
abs(x - 0.3) < 1e-16
```

But if we add 0.25 and 0.125, we get exact results.

```{code-cell} ipython3
x = 0.25 + 0.125
x == 0.375
```

+++ {"deletable": false, "editable": false}

#### Question 3
 Why cannot we get exact result when we add 0.2 and 0.1, but we do get exact results for 0.25 and 0.125? Read the documentation for using printing floating point numbers [here](https://docs.python.org/3/library/stdtypes.html#old-string-formatting) or [here](https://docs.python.org/3/reference/lexical_analysis.html#f-strings). Print the number 0.1 used above to 5 and 20 decimal spaces to see how they are actually represented. The output should look like this:
```
0.1 = 0.10000
0.1 = 0.10000000000000000555
```

Further information:
1. https://en.wikipedia.org/wiki/Double-precision_floating-point_format
2. https://www.h-schmidt.net/FloatConverter/IEEE754.html
3. https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html

```{code-cell} ipython3
---
nbgrader:
  cell_type: markdown
  checksum: 10cffbb806f50f9efce3b5cb4cf7e7ff
  grade: true
  grade_id: cell-6c7b64b4e6a14cc1
  locked: false
  points: 20
  schema_version: 3
  solution: true
  task: false
slideshow:
  slide_type: fragment
---
%%capture q3output
# Print 0.1 to 5 decimal places
...
# Print 0.1 to 20 decimal places
...
```

```{code-cell} ipython3
q3output.show()
```

```{code-cell} ipython3
# The following test should pass
def test_q3output(q3output):
    expected_q3output = """\
0.1 = 0.10000
0.1 = 0.10000000000000000555
"""
    assert q3output.stdout == expected_q3output
test_q3output(q3output)
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q3")
```

+++ {"deletable": false, "editable": false}

#### Question 4
Read about [64-bit IEEE 754 floating point format](https://en.wikipedia.org/wiki/Double-precision_floating-point_format). Find the difference between closest two binary numbers near 0.3 decimal that can be exactly represented in 64-bit IEEE 754 floating point format. Check your answer by comparing `abs((0.1 + 0.2) - 0.3) <= your_answer`

Hint: You have to only find the exponent part, e. The difference between closest binary numbers due to the fraction part $f$ is always $2^{-52}$ because $f$ is represented using 52 bits. Once you know the exponent, the difference is $2^{-52 + e}$. You can use `2**(-52+e)` to find the power of 2.

```{code-cell} ipython3
your_answer = ...
```

```{code-cell} ipython3
def test_floating_point(your_answer):
    assert abs(0.1+0.2-0.3) <= your_answer

test_floating_point(your_answer)
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q4")
```

+++ {"id": "r2A9ApyaL9fB", "slideshow": {"slide_type": "slide"}}

##### Differences from C
Note that unlike many languages, **Python does not have unary increment** (x++) or decrement (x--) operators.

Python also has built-in types for complex numbers; you can find all of the details in the [Builtin types documentation](https://docs.python.org/3.8/library/stdtypes.html#numeric-types-int-float-long-complex).

+++ {"id": "Nv_LIVOJL9fD", "slideshow": {"slide_type": "slide"}}

#### Booleans

Python implements all of the usual operators for Boolean logic, but uses English words rather than symbols (`&&`, `||`, etc.):

```{code-cell} ipython3
---
id: RvoImwgGL9fE
slideshow:
  slide_type: fragment
---
t = True; f = False
print(type(t))
```

+++ {"id": "YQgmQfOgL9fI", "slideshow": {"slide_type": "skip"}}

Now we let's look at the operations:

```{code-cell} ipython3
---
id: 6zYm7WzCL9fK
slideshow:
  slide_type: slide
---
print(t and f) # Logical AND;
print(t or f)  # Logical OR;
print(not t)   # Logical NOT;
print(t != f)  # Logical XOR;
```

+++ {"slideshow": {"slide_type": "slide"}}

#### NoneType
`None` constant can be thought of has `nullptr` in C. It has its own type `NoneType`.

```{code-cell} ipython3
---
slideshow:
  slide_type: fragment
---
n = None
type(None), type(n)
```

+++ {"id": "UQnQWFEyL9fP", "slideshow": {"slide_type": "slide"}}

### Sequences

* **str** (strings): usually to represent text. However, anything that is wrapped in quotes (single or double) is treated as a string.
* **list** (lists): Like a C array, more like C++ std::vector. Dynamically allocated and memory managed.
* **tuple** : Like lists but immutable.
* **set**: Like lists, mutable. Checks for uniqueness of each value.

+++ {"slideshow": {"slide_type": "slide"}}

#### Strings

```{code-cell} ipython3
---
id: AijEDtPFL9fP
slideshow:
  slide_type: fragment
---
hello = 'he"llo'   # String literals can use single quotes
world = "world'"   # or double quotes; it does not matter
great = """Python
also has triple quotes
which can internally contain
newlines, 'single quotes' and "double quotes".
Triple quotes are often used for documentation
and multi-line comments.
"""
print(hello)
print(len(hello))
```

```{code-cell} ipython3
---
id: saDeaA7hL9fT
slideshow:
  slide_type: fragment
---
hw = hello + ', ' + world  # String concatenation
print(hw)
```

```{code-cell} ipython3
---
id: Nji1_UjYL9fY
slideshow:
  slide_type: fragment
---
num = 12
hw12 = f'{hello:s} {world} {num:d}'  # string formatting
hw12
```

```{code-cell} ipython3
---
slideshow:
  slide_type: slide
---
"First, thou shalt count to {0}"  # References first positional argument
"Bring me a {}"                   # Implicitly references the first positional argument
"From {} to {}"                   # Same as "From {0} to {1}"
"My quest is {name}"              # References keyword argument 'name'
"Weight in tons {0.weight}"       # 'weight' attribute of first positional arg
"Units destroyed: {players[0]}"   # First element of keyword argument 'players'.
```

+++ {"id": "oMUHQqm6XR27", "slideshow": {"slide_type": "slide"}}

String formatting has its [own mini language](https://docs.python.org/3.8/library/string.html#formatspec).

<pre>
<strong id="id1">format_spec    </strong> ::=  [[<a class="reference internal" href="#grammar-token-fill"><code class="xref docutils literal notranslate"><span class="pre">fill</span></code></a>]<a class="reference internal" href="#grammar-token-align"><code class="xref docutils literal notranslate"><span class="pre">align</span></code></a>][<a class="reference internal" href="#grammar-token-sign"><code class="xref docutils literal notranslate"><span class="pre">sign</span></code></a>][#][0][<a class="reference internal" href="#grammar-token-width"><code class="xref docutils literal notranslate"><span class="pre">width</span></code></a>][<a class="reference internal" href="#grammar-token-grouping-option"><code class="xref docutils literal notranslate"><span class="pre">grouping_option</span></code></a>][.<a class="reference internal" href="#grammar-token-precision"><code class="xref docutils literal notranslate"><span class="pre">precision</span></code></a>][<a class="reference internal" href="#grammar-token-type"><code class="xref docutils literal notranslate"><span class="pre">type</span></code></a>]
<strong id="grammar-token-fill">fill           </strong> ::=  &lt;any character&gt;
<strong id="grammar-token-align">align          </strong> ::=  &quot;&lt;&quot; | &quot;&gt;&quot; | &quot;=&quot; | &quot;^&quot;
<strong id="grammar-token-sign">sign           </strong> ::=  &quot;+&quot; | &quot;-&quot; | &quot; &quot;
<strong id="grammar-token-width">width          </strong> ::=  <a class="reference internal" href="../reference/lexical_analysis.html#grammar-token-digit"><code class="xref docutils literal notranslate"><span class="pre">digit</span></code></a>+
<strong id="grammar-token-grouping-option">grouping_option</strong> ::=  &quot;_&quot; | &quot;,&quot;
<strong id="grammar-token-precision">precision      </strong> ::=  <a class="reference internal" href="../reference/lexical_analysis.html#grammar-token-digit"><code class="xref docutils literal notranslate"><span class="pre">digit</span></code></a>+
<strong id="grammar-token-type">type           </strong> ::=  &quot;b&quot; | &quot;c&quot; | &quot;d&quot; | &quot;e&quot; | &quot;E&quot; | &quot;f&quot; | &quot;F&quot; | &quot;g&quot; | &quot;G&quot; | &quot;n&quot; | &quot;o&quot; | &quot;s&quot; | &quot;x&quot; | &quot;X&quot; | &quot;%&quot;
</pre>

```{code-cell} ipython3
---
id: a7DAK5bmWtgP
slideshow:
  slide_type: slide
---
hw12 = '%s %s %d' % (hello, world, 12) # C-style string formatting using % operator
hw12
```

+++ {"id": "Ll6wRsO4ZS1h", "slideshow": {"slide_type": "slide"}}

Since Python 3.8, the dominant way to format strings is to use [f-string](https://docs.python.org/3/glossary.html#term-f-string)

```{code-cell} ipython3
---
id: 38_U-Hg7YxMI
slideshow:
  slide_type: fragment
---
hello = "Hell'o"
world = 'World"'
i = 12
hw12 = F'{hello:s} {world} {i:d}'# string formatting using f-strings
hw12
```

```{code-cell} ipython3
---
id: BN6Ka_cyacaL
slideshow:
  slide_type: skip
---
hw12 = hello + world + str(12)
print(hw12)
```

```{code-cell} ipython3
---
id: vFqZ-fFC_KxZ
slideshow:
  slide_type: skip
---
hw12 = hello + ' ' + world + ' ' + str(3.1415)
print(hw12)
```

```{code-cell} ipython3
---
id: 4tjJcT_eaFOm
slideshow:
  slide_type: skip
---
# Yes you can use emojis (unicode support)
f"{hello} 😀 😃 😄 😁 😆 {i:d} emojies"
```

+++ {"id": "CsJzWurrahjk", "slideshow": {"slide_type": "skip"}}

Or you can use [Unicode names for emojies](https://docs.python.org/3/howto/unicode.html)

```{code-cell} ipython3
---
id: W8gBwi4Eatm6
slideshow:
  slide_type: skip
---
"\N{GRINNING FACE}"
```

+++ {"id": "bUpl35bIL9fc", "slideshow": {"slide_type": "slide"}}

String objects have a bunch of useful methods; for example:

```{code-cell} ipython3
---
id: VOxGatlsL9fd
slideshow:
  slide_type: fragment
---
s = "hello"
print(s.capitalize())  # Capitalize a string
print(s.upper())       # Convert a string to uppercase; prints "HELLO"
print(s.rjust(7))      # Right-justify a string, padding with spaces
print(s.center(7))     # Center a string, padding with spaces
print(s.replace('l', '(ell)'))  # Replace all instances of one substring with another
```

```{code-cell} ipython3
---
id: 3TPiIvkN_z98
slideshow:
  slide_type: skip
---
s = 'hello'
s = s.upper()
print(s)
```

+++ {"id": "vF00P7btXiSO", "slideshow": {"slide_type": "slide"}}

You can ask for help on any python function, object or type using "?"

```{code-cell} ipython3
---
id: wm6gBA0BXffY
slideshow:
  slide_type: fragment
---
 # a jupyter notebook feature, not python
s.upper?
```

```{code-cell} ipython3
---
id: 6DASciYCX4DQ
slideshow:
  slide_type: fragment
---
help(s.upper) # Some as s.upper? but a python feature
```

```{code-cell} ipython3
---
id: Hpy-ZDCZYJPk
slideshow:
  slide_type: fragment
---
dir(s) # List all the functions available on a string object
```

+++ {"id": "CRT0QMj3PbGj", "slideshow": {"slide_type": "skip"}}

**Whitespace**: any nonprinting character, such as spaces, tabs, and end-of-line symbols

```{code-cell} ipython3
---
id: WowvLGAfPqW6
slideshow:
  slide_type: skip
---
print("ECE 491/591\n\tDeep Learning\nis fun!")
```

```{code-cell} ipython3
---
id: xy-5QHb2PZx_
slideshow:
  slide_type: skip
---
print('  w\to\tr  l\nd '.strip())  # Strip leading and trailing whitespace
```

```{code-cell} ipython3
---
id: zkohEUdQQL2w
slideshow:
  slide_type: skip
---
print('  world '.rstrip())  # Strip trailing whitespace
```

```{code-cell} ipython3
---
id: fdxUM68hQie0
slideshow:
  slide_type: skip
---
print('  world '.lstrip())  # Strip leading whitespace
```

+++ {"id": "06cayXLtL9fi", "slideshow": {"slide_type": "skip"}}

You can find a list of all string methods in the [documentation](https://docs.python.org/3.7/library/stdtypes.html#string-methods).

+++ {"id": "S3tkgmsvBKd9", "slideshow": {"slide_type": "slide"}}

#### Difference from C
C is statically typed vs Python is dynamically typed. You can change the type of a variable from one line to another.

```{code-cell} ipython3
---
id: Bpgx2b-c-63n
slideshow:
  slide_type: fragment
---
x = 1
print("1. Type of x = ", type(x))
x = "str"
print("2. Type of x = ", type(x))
```

+++ {"id": "UsIWOe0LL9fn", "slideshow": {"slide_type": "skip"}}

#### Lists

+++ {"id": "wzxX7rgWL9fn", "slideshow": {"slide_type": "fragment"}}

A list is the Python equivalent of an array, but is resizeable and can contain elements of different types:

```{code-cell} ipython3
---
id: QmHcwNjubev4
slideshow:
  slide_type: fragment
---
lst = [1, 2, 3, 4, 5]
print(lst)
```

+++ {"id": "YMnnw-K-coqI", "slideshow": {"slide_type": "slide"}}

**Index Positions Start at 0, Not 1**

```{code-cell} ipython3
---
id: dvdKRR5Sb15z
slideshow:
  slide_type: fragment
---
lst[0]
```

```{code-cell} ipython3
---
id: hk3A8pPcL9fp
slideshow:
  slide_type: slide
---
xs = [1, 2, 3, 'hello', [4, 5, 6]]    # Create a list
print(xs)
print('First element of the array: ', xs[0])  # Index to the list starts with 0
print('Last element of the array: ', xs[4])
print(xs[4][1])
```

```{code-cell} ipython3
---
id: u-SeyJugSnXm
slideshow:
  slide_type: fragment
---
# Negative indices count from the end of the list;
print(xs[-1])     # Index -1 returns the last item in the list
print(xs[-2])     # Index -2 returns the second item from the end of the list
```

```{code-cell} ipython3
---
id: YCjCy_0_L9ft
slideshow:
  slide_type: slide
---
xs[2] = 'foo'    # Lists can contain elements of different types
print(xs)
```

```{code-cell} ipython3
---
id: vJ0x5cF-L9fx
slideshow:
  slide_type: fragment
---
xs.append('bar') # Add a new element to the end of the list
print(xs)
```

```{code-cell} ipython3
---
id: cxVCNRTNL9f1
slideshow:
  slide_type: slide
---
print(xs)
x = xs.pop()     # Remove and return the last element of the list
print(x)
print(xs)
```

```{code-cell} ipython3
---
id: Kq5WHOEzTKYm
slideshow:
  slide_type: fragment
---
xs.insert(1, "new item") # Insert the "new item" at the index 1
print(xs)
```

```{code-cell} ipython3
---
id: rUZ2UU1MTx_y
slideshow:
  slide_type: fragment
---
xs.remove("foo") # Removing an item by value
print(xs)
```

+++ {"id": "ilyoyO34L9f4", "slideshow": {"slide_type": "slide"}}

As usual, you can find all the details about lists in the [documentation](https://docs.python.org/3.8/tutorial/datastructures.html#more-on-lists).

+++ {"deletable": false, "editable": false, "id": "ilyoyO34L9f4", "slideshow": {"slide_type": "slide"}}

<!-- BEGIN QUESTION -->

#### Question 5
Read the above linked list documentation to find out whether Python lists are implemented as array datastructure or linked list datastructure.

+++

_Type your answer here, replacing this text._

+++ {"deletable": false, "editable": false}

<!-- END QUESTION -->


```{code-cell} ipython3
---
id: 0UN6AzpaB-yp
slideshow:
  slide_type: slide
---
colors = ["Red", "Green", "White", "Black"]
# Print the first color in the above list
print(colors[0])
# Print the last color in the above list
print(colors[-1])
# Append "Blue" to the above list
colors.append('Blue')
print(colors)
# Remove "Green" from the above list
colors.remove('Green')
print(colors)
```

+++ {"deletable": false, "editable": false, "id": "4qR6pXLXe2uh", "slideshow": {"slide_type": "fragment"}}

#### Question 6
Find the length of the list `colors`.

```{code-cell} ipython3
---
id: TLACKBuUe92y
nbgrader:
  cell_type: code
  checksum: 15e81c33179fd07bcc602ce458d35999
  grade: false
  grade_id: cell-711b6cbf20d544c0
  locked: false
  schema_version: 3
  solution: true
  task: false
slideshow:
  slide_type: skip
---
def length_of_colors(colors):
    colors_lengths = ...
    return colors_length
```

```{code-cell} ipython3
def test_length_of_colors(length_of_colors):
    import random
    n = random.randint(1, 100)
    colors = ['red'] * n
    assert n == len(colors)
test_length_of_colors(length_of_colors)
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q6")
```

```{code-cell} ipython3
---
nbgrader:
  cell_type: code
  checksum: f6cdb3c8723657e22245b94cc8bfd859
  grade: true
  grade_id: cell-08d951a4a5fb5045
  locked: true
  points: 5
  schema_version: 3
  solution: false
  task: false
slideshow:
  slide_type: skip
---
# Last cell should output length of the list `colors`
```

+++ {"deletable": false, "editable": false, "id": "vwbt_pirA6Gv", "slideshow": {"slide_type": "slide"}}

#### Question 7

What is the difference between sorted(colors) and colors.sort()?

```{code-cell} ipython3
---
deletable: false
editable: false
id: OC9qQkgOfi7g
slideshow:
  slide_type: fragment
---
colors = ["Red", "Green", "White", "Black"]
colors2 = sorted(colors)
colors2, colors
```

```{code-cell} ipython3
---
deletable: false
editable: false
id: VkaIRgQKBOg9
slideshow:
  slide_type: fragment
---
colors.sort()
colors
```

Your answer here

+++ {"id": "ovahhxd_L9f5", "slideshow": {"slide_type": "slide"}}

#### Slicing

+++ {"id": "YeSYKhv9L9f6", "slideshow": {"slide_type": "fragment"}}

In addition to accessing list elements one at a time, Python provides concise syntax to access sublists; this is known as slicing:

* aList[**start:stop:step**]
* aList[**start:stop**]

The default for **start** is none or 0.

The default **stop** is the end of your data structure.

Using a positive number references from the first element, a negative number references from last element in your structure.

```{code-cell} ipython3
---
id: ninq666bL9f6
slideshow:
  slide_type: slide
---
# nums = range(5) # Python 2, but error in Python 3
nums = list(range(5))    # range is a built-in function that creates a list of integers
nums = [0, 1, 2, 3, 4]
print(nums)         # Prints "[0, 1, 2, 3, 4]"
print(nums[2:4])    # Get a slice from index 2 to 4 (exclusive); prints "[2, 3]"
print(nums[2:])     # Get a slice from index 2 to the end; prints "[2, 3, 4]"
print(nums[:2])     # Get a slice from the start to index 2 (exclusive); prints "[0, 1]"
print(nums[:])      # Get a slice of the whole list; prints ["0, 1, 2, 3, 4]"
print(nums[::-1])    # Slice indices can be negative; prints ["0, 1, 2, 3]"
nums[2:4] = [8, 9]  # Assign a new sublist to a slice
print(nums)         # Prints "[0, 1, 8, 9, 4]"
```

```{code-cell} ipython3
---
id: 4N1ao1fbC6gs
slideshow:
  slide_type: slide
---
lst = list(range(0,100,10))
print(lst)
```

```{code-cell} ipython3
---
id: UmOA19VODKnO
slideshow:
  slide_type: fragment
---
# Print everything except the last three elements
print(lst[:-3])
```

```{code-cell} ipython3
---
id: u3R02qZaDPHu
slideshow:
  slide_type: fragment
---
# Print everything with odd indices
print(lst[::2])
```

```{code-cell} ipython3
---
id: 0aEcIpN5D6gs
slideshow:
  slide_type: slide
---
# Print everything with even indices
print(lst[1::2])
```

```{code-cell} ipython3
---
id: 3cv_b7esEDkH
slideshow:
  slide_type: fragment
---
# Print everything in reversed order
print(lst[::-1])
```

+++ {"deletable": false, "editable": false, "id": "4RRJ3mEvCmUK", "slideshow": {"slide_type": "slide"}}

#### Question 8

Select 7th, 5th, and 3th element from the following list `A` using a single list slice

```{code-cell} ipython3
---
deletable: false
editable: false
id: l_gYn32_hccI
slideshow:
  slide_type: fragment
---
A = list(range(1,10,1)) # start,stop,step
print(A)
```

```{code-cell} ipython3
---
id: mzbPvkFzjWMl
nbgrader:
  cell_type: code
  checksum: d4624a0a94e655f38a86f291b381cf14
  grade: false
  grade_id: cell-11e1de3272672679
  locked: false
  schema_version: 3
  solution: true
  task: false
slideshow:
  slide_type: fragment
---
def ele_7th_to_3rd_list(A):
    slice_7th_to_3rd = ...
    return slice_7th_to_3rd
ele_7th_to_3rd_list(A)
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q8")
```

```{code-cell} ipython3
---
editable: false
nbgrader:
  cell_type: code
  checksum: 261123a94a46d06185310a9923673855
  grade: true
  grade_id: cell-77ef2501dad25981
  locked: true
  points: 5
  schema_version: 3
  solution: false
  task: false
slideshow:
  slide_type: fragment
---
# Last cell should output length of the list `colors`
```

+++ {"id": "UONpMhF4L9f_", "slideshow": {"slide_type": "slide"}}

#### Loops

+++ {"id": "_DYz1j6QL9f_", "slideshow": {"slide_type": "-"}}

You can loop over the elements of a list like this:

```{code-cell} ipython3
---
id: 4cCOysfWL9gA
slideshow:
  slide_type: '-'
---
animals = ['cat', 'dog', 'monkey']
for animal in animals:
  print(animal)
```

+++ {"slideshow": {"slide_type": "slide"}}

You can print squares of first 20 numbers like this:

```{code-cell} ipython3
---
slideshow:
  slide_type: '-'
---
for i in range(1, 21):
  print(f"{i:d}^2 = {i*i:d}")
```

+++ {"id": "KxIaQs7pL9gE", "slideshow": {"slide_type": "slide"}}

##### Differences from C
Unlike C there are no braces for start and end of the for loop. Instead of braces, we have a compbination of ":" colon and indentation. Unlike C, indentation is mandatory in Python.

If you want access to the index of each element within the body of a loop, use the built-in `enumerate` function:

```{code-cell} ipython3
---
id: JjGnDluWL9gF
slideshow:
  slide_type: skip
---
animals = ['cat', 'dog', 'monkey']
for idx, animal in enumerate(animals):
    print('#{}: {}'.format(idx, animal))
```

+++ {"deletable": false, "editable": false, "slideshow": {"slide_type": "slide"}}

#### Question 9

Write code to create a list of 20 elements of fibonacci series: $f_0 = 0$, $f_1 = 1$, and $f_n = f_{n-1} + f_{n-2}$ for all $n \ge 2$. Assign the list to variable called FIB20.

```{code-cell} ipython3
---
nbgrader:
  cell_type: code
  checksum: 454b6f27a3b14d728dcbe6213ce76eb6
  grade: false
  grade_id: cell-a524476b59e11486
  locked: false
  schema_version: 3
  solution: true
  task: false
slideshow:
  slide_type: skip
---
# Fibonacci series
FIB20 = [] # modify the list to contain fibonacci using a for loop
...
print(FIB20)
```

```{code-cell} ipython3
---
nbgrader:
  cell_type: code
  checksum: 3866d385712294577175e765647d07cb
  grade: true
  grade_id: cell-29cf37a9b9e5da65
  locked: true
  points: 20
  schema_version: 3
  solution: false
  task: false
slideshow:
  slide_type: skip
---
assert len(FIB20) == 20
assert FIB20[0] == 0
assert FIB20[1] == 1
### START HIDDEN TESTS
assert FIB20 == [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181]
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q9")
```

+++ {"id": "7761hhtzeIrJ", "slideshow": {"slide_type": "skip"}}

#### Copying a list

+++ {"id": "X4iruzPrfOWR", "slideshow": {"slide_type": "skip"}}

To copy a list, you can make a slice that includes the entire original list
by omitting the first index and the second index ([:]). This tells Python to
make a slice that starts at the first item and ends with the last item, producing
a copy of the entire list.

```{code-cell} ipython3
---
id: NH6ayWFdeHYU
slideshow:
  slide_type: skip
---
a = [1, 2, 3, 4, 5]
b = a[:]  # copying the list
# b = a.copy() # <- preferred way
c = a     # This doesn't copy the list!

a.append('x') # a = [1, 2, 3, 4, 5, 'x']
b.append('y') # b = [1, 2, 3, 4, 5, 'y']
c.append('z') # c = [1, 2, 3, 4, 5, 'x', 'z']

print('a = ', a)
print('b = ', b)
print('c = ', c)
print('a = ', a)
```

+++ {"id": "arrLCcMyL9gK", "slideshow": {"slide_type": "slide"}}

#### List comprehensions:

+++ {"id": "5Qn2jU_pL9gL", "slideshow": {"slide_type": "-"}}

When programming, frequently we want to transform one type of data into another. As a simple example, consider the following code that computes square numbers:

```{code-cell} ipython3
---
id: IVNEwoMXL9gL
slideshow:
  slide_type: slide
---
nums = [0, 1, 2, 3, 4]
squares = []
for x in nums: # Don't forget the colon
    squares.append(x ** 2)
print(squares)
```

+++ {"id": "7DmKVUFaL9gQ", "slideshow": {"slide_type": "-"}}

You can make this code simpler using a list comprehension:

```{code-cell} ipython3
---
id: kZxsUfV6L9gR
slideshow:
  slide_type: fragment
---
nums = [0, 1, 2, 3, 4]
def sq(x): return x*x
squares = list(map(sq, nums))
print(squares)
```

+++ {"id": "-D8ARK7tL9gV", "slideshow": {"slide_type": "slide"}}

List comprehensions can also contain conditions:

```{code-cell} ipython3
---
id: yUtgOyyYL9gV
slideshow:
  slide_type: '-'
---
nums = [0, 1, 2, 3, 4]
def filter_cond(x):
    return (x % 2 == 0 and x <=2)
even_squares = list(map(sq, filter(filter_cond,nums)))
print(even_squares)
```

+++ {"id": "H8xsUEFpL9gZ", "slideshow": {"slide_type": "slide"}}

#### Dictionaries

+++ {"id": "kkjAGMAJL9ga", "slideshow": {"slide_type": "-"}}

A dictionary stores (key, value) pairs, similar to a `Map` in Java or an object in Javascript. It is implemented as a [hash table](https://en.wikipedia.org/wiki/Hash_table).

![](imgs/hash-table.png)

You can use it like this:

```{code-cell} ipython3
hash('dog') % 1
```

```{code-cell} ipython3
---
id: XBYI1MrYL9gb
slideshow:
  slide_type: slide
---
d = {'cat': 'cute', # { key : value, }
     'dog': 'furry'}  # Create a new dictionary with some data
print(d['cat'])       # Get an entry from a dictionary; prints "cute"
key = 'cat'
print(d[key])
print('cat' in d)     # Check if a dictionary has a given key; prints "True"
```

+++ {"id": "4_bPWZAfhwLv", "slideshow": {"slide_type": "slide"}}

Adding a new key-value pair

```{code-cell} ipython3
---
id: pS7e-G-HL9gf
slideshow:
  slide_type: '-'
---
d['fish'] = 'wet'    # Set an entry in a dictionary
print(d['fish'])     # Prints "wet"

d['elephant'] = 'heavy'  # Set an entry in a dictionary
print(d['elephant'])     # Prints "heavy"
```

+++ {"id": "zy1_tEtqj0Rv", "slideshow": {"slide_type": "slide"}}

Looping through a dictionary

```{code-cell} ipython3
---
id: yngQ8bQfj3sQ
slideshow:
  slide_type: '-'
---
for key, value in d.items():
  print('key = ', key, '\t value = ', value)
```

```{code-cell} ipython3
---
id: gMynGQKEkcn_
slideshow:
  slide_type: fragment
---
for key in d.keys():
  print('key = ', key, '\tvalue = ', d[key])
```

```{code-cell} ipython3
---
id: oU6QMrO-k2B0
slideshow:
  slide_type: slide
---
for key in sorted(d.keys()):  # sorting the key
  print('key = ', key, '\t\tvalue = ', d[key])
```

```{code-cell} ipython3
---
id: hwnFSEeTlbK-
slideshow:
  slide_type: fragment
---
for value in sorted(d.values()):  # print all values
  print('value = ', value)
```

```{code-cell} ipython3
---
id: tFY065ItL9gi
slideshow:
  slide_type: slide
---
print('monkey' in d)

# print(d['monkey'])  # KeyError: 'monkey' not a key of d
```

+++ {"id": "AV3EoVzejUj2", "slideshow": {"slide_type": "slide"}}

For dictionaries, we can use the get() method to set a default value that will be returned if the requested key doesn’t exist.

```{code-cell} ipython3
---
id: 8TjbEWqML9gl
slideshow:
  slide_type: '-'
---
print(d.get('monkey', 'N/A'))  # Get an element with a default; prints "N/A"
print(d.get('fish', 'N/A'))    # Get an element with a default; prints "wet"
```

+++ {"id": "Nk2wF6aRiG1X", "slideshow": {"slide_type": "slide"}}

Removing a key-value pair

```{code-cell} ipython3
---
id: 0EItdNBJL9go
slideshow:
  slide_type: '-'
---
del d['fish']        # Remove an element from a dictionary

# Be aware that the deleted key-value pair is removed permanently.
print(d.get('fish', 'N/A')) # "fish" is no longer a key; prints "N/A"
```

+++ {"id": "wqm4dRZNL9gr", "slideshow": {"slide_type": "fragment"}}

You can find all you need to know about dictionaries in the [documentation](https://docs.python.org/2/library/stdtypes.html#dict).

+++ {"id": "IxwEqHlGL9gr"}

It is easy to iterate over the keys in a dictionary:

```{code-cell} ipython3
:id: rYfz7ZKNL9gs

d = {'person': 2, 'cat': 4, 'spider': 8}
for animal, legs in d.items():
    print('A {} has {} legs'.format(animal, legs))
```

+++ {"id": "17sxiOpzL9gz"}

Dictionary comprehensions: These are similar to list comprehensions, but allow you to easily construct dictionaries. For example:

```{code-cell} ipython3
---
id: 8PB07imLL9gz
slideshow:
  slide_type: slide
---
nums = [0, 1, 2, 3, 4]
even_num_to_square = {x: x ** 2 for x in nums if x % 2 == 0}
print(even_num_to_square)
```

+++ {"id": "qPsHSKB1L9hF", "slideshow": {"slide_type": "slide"}}

#### Tuples

+++ {"id": "kucc0LKVL9hG", "slideshow": {"slide_type": "-"}}

A tuple is an (immutable) ordered list of values. Python refers to values that cannot change as immutable, and an immutable list is called a tuple.

```{code-cell} ipython3
---
id: 0R0xAUTigZ7f
slideshow:
  slide_type: fragment
---
dimensions = (800, 600)
print(dimensions[0])
print(dimensions[1])
```

+++ {"id": "RTCP4icsCN-N", "slideshow": {"slide_type": "fragment"}}

Note that comma creates a tuple not parantheses. Always use parantheses to increase readability.

```{code-cell} ipython3
:id: rbMZWkk7CUEk

dimensions = 800, 600
print(dimensions[0])
print(dimensions[1])
```

```{code-cell} ipython3
:id: 3E_SuJaGgnvg

# Try this (uncomment)
# dimensions[0] = 1000  # this will return error
```

+++ {"id": "x7Dem8CxgZEz"}

A tuple is in many ways similar to a list; one of the most important differences is that tuples can be used as keys in dictionaries and as elements of sets, while lists cannot. Here is a trivial example:

```{code-cell} ipython3
:id: 9wHUyTKxL9hH

d = {(x, x + 1): x for x in range(10)}  # Create a dictionary with tuple keys
t = (5, 6)       # Create a tuple
print(type(t))
print(d[t])
print(d[(1, 2)])
```

+++ {"id": "gHw9hjSgClmz", "slideshow": {"slide_type": "slide"}}

Any sequence (list, tuple, set) can be _unpacked_ into individual variables.

```{code-cell} ipython3
---
id: 3s_OArLXRo0P
slideshow:
  slide_type: '-'
---
dimensionx, dimensiony = dimensions
```

```{code-cell} ipython3
---
id: 5XKKRCGsDPk6
slideshow:
  slide_type: slide
---
x, y, z = [1, 2, 3] # Lists can be used for Multiple assignment
print(x, y, z)
```

```{code-cell} ipython3
---
id: O_Aak5bADf5Q
slideshow:
  slide_type: slide
---
x, y, *rest = range(10) # Lists can be used for Multiple assignment
x, y, rest
```

```{code-cell} ipython3
---
id: azrHHC0-DH4t
slideshow:
  slide_type: slide
---
x, y, z = 1, 2, 3  # Tuples can be used for Multiple assignment
print(x, y, z)
```

+++ {"id": "K9f25_1jlNRF", "slideshow": {"slide_type": "slide"}}

### Flow Control

```{code-cell} ipython3
---
id: a2EwpTgQmayj
slideshow:
  slide_type: '-'
---
fruits = ["apple", "banana", "cherry"]
for x in fruits:
  print(x)
```

```{code-cell} ipython3
---
id: vt5SHmnzmgPR
slideshow:
  slide_type: fragment
---
for x in "banana": # string is also a sequence type
  print(x)
```

```{code-cell} ipython3
---
id: rzJWiU0jmj3c
slideshow:
  slide_type: slide
---
fruits = ["apple", "banana", "cherry"]
for x in fruits:
  if x == "banana":
    break
  print(x)
```

```{code-cell} ipython3
---
id: 9gbZ7NtLmpMd
slideshow:
  slide_type: fragment
---
fruits = ["apple", "banana", "cherry"]
for x in fruits:
  if x == "banana":
    continue
  print(x)
```

```{code-cell} ipython3
---
id: UXfW14pKc_Ab
slideshow:
  slide_type: slide
---
# Shows while loop. Do not use this style though, prefer for loop.
# This style is considered not Pythonic
# https://peps.python.org/pep-0008/
idx = 0
while (idx < len(fruits)):
  print(fruits[idx])
  idx += 1
```

```{code-cell} ipython3
---
id: CXMuxRe2ldUK
slideshow:
  slide_type: slide
---
age = 19
if age >= 18:
  print("You are old enough to vote!")
else:
  print("Sorry, you are too young to vote.")
```

```{code-cell} ipython3
---
id: K_cH-E6sl02v
slideshow:
  slide_type: slide
---
age = 12
if age < 4:
  price = 0
elif age < 18:
  price = 25
else:
  price = 40

print(f"Your admission cost is ${price}.")
```

+++ {"id": "1ASYoQRycx2T", "slideshow": {"slide_type": "slide"}}

### Differences from C

There is no `do {} while()` in Python.

+++ {"slideshow": {"slide_type": "-"}}

#### Truth value testing

Any objects can be tested for truth value in `if` and `while` statements. Most objects are considered true, except the following ones which are considered false:

* constants defined to be false: None and False.

* zero of any numeric type: 0, 0.0, 0j, Decimal(0), Fraction(0, 1)

* empty sequences and collections: '', (), [], {}, set(), range(0)

+++ {"id": "AXA4jrEOL9hM", "slideshow": {"slide_type": "slide"}}

### Functions

+++ {"id": "WaRms-QfL9hN", "slideshow": {"slide_type": "-"}}

Python functions are defined using the `def` keyword. For example:

```{code-cell} ipython3
---
id: RkuRguV-jvP5
slideshow:
  slide_type: '-'
---
# Making the argument b and c optional with 4 and 8 as default values
def sum2(a, b=4, c=8):
  return a + b + c, a*b*c # Returns a tuple of two values
```

```{code-cell} ipython3
---
id: Bbsc-8Ivj3bY
slideshow:
  slide_type: '-'
---
sum2(1, c=5)
```

```{code-cell} ipython3
---
id: pKf2_UzqkwNa
slideshow:
  slide_type: slide
---
sum_, prod_ = sum2(1, c=5) # Unpacking a tuple
```

```{code-cell} ipython3
---
id: oZE6AoRvk77-
slideshow:
  slide_type: '-'
---
prod_
```

```{code-cell} ipython3
---
id: kiMDUr58L9hN
slideshow:
  slide_type: slide
---
def sign(x, y=100):
  if x > 0:
    return 'positive', 123
  elif x < 0:
    return 'negative', 123
  else:
    return 'zero', 123

for x in [-1, 0, 1]:
  v1, _ = sign(x)
  print(v1)
```

+++ {"id": "U-QJFt8TL9hR", "slideshow": {"slide_type": "slide"}}

We will often define functions to take [optional keyword](https://docs.python.org/3/glossary.html#term-keyword-argument) arguments, like this:

```{code-cell} ipython3
---
id: PfsZ3DazL9hR
slideshow:
  slide_type: '-'
---
def hello(name, loud=False):
    if loud:
        print('HELLO, {}'.format(name.upper()))
    else:
        print('Hello, {}!'.format(name))

hello('Bob')
hello('Fred', loud=True)
```

+++ {"id": "kHknme7EocwW", "slideshow": {"slide_type": "slide"}}

Modifing a list in a function

When you pass a list to a function, the function can modify the list.

```{code-cell} ipython3
---
id: CdIUyKhMobjw
slideshow:
  slide_type: '-'
---
def fun(list):
  list.append('new item')

a = [1, 2, 3]
fun(a)
print(a)
```

```{raw-cell}
---
id: KZOZ7eKXpH5b
slideshow:
  slide_type: slide
---
If you want to prevent a function from modifying a list, you can send a copy of a list to the function
```

```{code-cell} ipython3
---
id: 7aT96a74pY6X
slideshow:
  slide_type: '-'
---
def fun(lst):
  lst.append('new item')

a = [1, 2, 3]
# Preventing a Function from Modifying a List
fun(a[:]) # The slice notation [:] makes a copy of the list
# fun(a.copy())
print(a)
```

+++ {"deletable": false, "editable": false, "id": "eHzBWi4UGJIq", "slideshow": {"slide_type": "slide"}}

#### Question 10

Write a function named "first_last" that returns a list that contains only the first and the last element of a given list

```{code-cell} ipython3
---
id: 4mmpsHIfGKJY
slideshow:
  slide_type: skip
---
def first_last(a):
    fst_last = ...
    return fst_last

first_last([1, 2, 3, 4, 5])
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q10")
```

+++ {"id": "baqShM2N8UWM", "slideshow": {"slide_type": "slide"}}

### Variable Scope

The LEGB scope lookup rule. When a variable is referenced, Python searches for it in
this order: in the local scope, in any enclosing functions’ local scopes, in the global scope, and finally
in the built-in scope. The first occurrence wins. The place in your code where a variable is assigned
usually determines its scope. In Python 3, nonlocal declarations can also force names to be mapped
to enclosing function scopes, whether assigned or not.

+++ {"id": "hXRCy21T9RN-", "slideshow": {"slide_type": "-"}}

![image.png](data:image/png;base64,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)

```{code-cell} ipython3
X = 99
def func():
    # new variable
    X = 88
    def func1():
        def func2():
            # new variable
            nonlocal X
            X = 66
        func2()
    func1()
    print(X)
func()
```

```{code-cell} ipython3
---
id: NLlLovU78W2Q
slideshow:
  slide_type: slide
---
# Global scope
X = 99 # X and func assigned in module: global

def func(Y): # Y and Z assigned in function: locals
  global X
  X = 11
  # Local scope
  Z = X + Y # X is a global
  print("Local X", X)
  for i in range(10):
    z = 8
  if True:
    flag = 'true'
  print(flag)
  return Z

func(1) # func in module: result=100
print("Global X", X)
```

```{code-cell} ipython3
---
id: RVZwE1-r91kH
slideshow:
  slide_type: slide
---
X = 99 # Global scope name: not used

def f1():
  X = 88 # Enclosing def local
  def f2():
    print(X) # Reference made in nested def
  f2()

f1() # Prints 88: enclosing def local
print(X)
```

```{code-cell} ipython3
---
id: YO1urliO8iE6
slideshow:
  slide_type: slide
---
X = 99 # Global X

def func():
  X = 88 # Local X: hides global

func()
print(X) # Prints Global X 99: unchanged
```

+++ {"id": "ryS19k3x8zjF", "slideshow": {"slide_type": "slide"}}

The assignment within the function creates a local X that is a completely different
variable from the global X in the module outside the function.

```{code-cell} ipython3
---
id: A3r-ho9z85rQ
slideshow:
  slide_type: '-'
---
X = 99 # Global X

def func():
  global X
  X = 88 # Local X: overrides global

func()
print(X) # Prints 88: unchanged
```

```{code-cell} ipython3
---
slideshow:
  slide_type: slide
---
for x in range(100):
    pass

# what is x
x
```

```{code-cell} ipython3
---
slideshow:
  slide_type: slide
---
x = 0
for i in range(100):
    if i % 23 == 0:
        x = 23
    else:
        x = 1
# what is x?
x
```

```{code-cell} ipython3
---
slideshow:
  slide_type: slide
---
x = 0
def foo():
    for i in range(100):
        if i % 23 == 0:
            x = 23
        else:
            x = 1
# what is x?
x
```

+++ {"deletable": false, "editable": false, "slideshow": {"slide_type": "slide"}}

#### Question 11 : Implement a sqrt function

As a way to play with functions and loops, let’s implement a square root function: given a number x, we want to find the number z for which z² is most nearly x.

Computers typically compute the square root of x using a loop. Starting with some guess z, we can adjust z based on how close z² is to x, producing a better guess:
$z' = z - (z^2 - x) / (2z) $

Repeating this adjustment makes the guess better and better until we reach an answer that is as close to the actual square root as can be.

Implement this in the def sqrt provided. A decent starting guess for z is 1, no matter what the input. To begin with, repeat the calculation 10 times and print each z along the way. See how close you get to the answer for various values of x (1, 2, 3, …) and how quickly the guess improves.

Next, change the loop condition to stop once the value has stopped changing (or only changes by a very small amount). See if that’s more or fewer than 10 iterations. Try other initial guesses for z, like x, or x/2. How close are your function’s results to the math.sqrt in the standard library?

(Note: If you are interested in the details of the algorithm, the z² − x above is how far away z² is from where it needs to be (x), and the division by 2z is the derivative of z², to scale how much we adjust z by how quickly z² is changing. This general approach is called Newton’s method. It works well for many functions but especially well for square root. Newton’s methods is particular form of gradient descent.)

```{code-cell} ipython3
---
nbgrader:
  cell_type: code
  checksum: 8fb1d90a265589f45eed07cf742bba6b
  grade: false
  grade_id: cell-30c08b69b69ff8c1
  locked: false
  schema_version: 3
  solution: true
  task: false
slideshow:
  slide_type: skip
---
def sqrt(x, threshold=1e-8):
    assert x >= 0 # throws an error if x is negative
    ...

sqrt(2)
```

```{code-cell} ipython3
---
nbgrader:
  cell_type: code
  checksum: fc015efafb9c9e9bc6cdc7dd8f046e86
  grade: true
  grade_id: cell-aaacc93f0f82ee73
  locked: true
  points: 20
  schema_version: 3
  solution: false
  task: false
slideshow:
  slide_type: slide
---
x = 2; z = sqrt(x)
assert abs(z*z - x) < 1e-8
x = 3; z = sqrt(x)
assert abs(z*z - x) < 1e-8
x = 4; z = sqrt(x)
assert abs(z*z - x) < 1e-8
import random
for _ in range(10):
    x = random.randint(1, 100); z = sqrt(x)
    assert abs(z*z - x) < 1e-6
'success'
```

```{code-cell} ipython3
:deletable: false
:editable: false

grader.check("q11")
```

+++ {"id": "ZKEfJ3e78ANJ", "slideshow": {"slide_type": "slide"}}

### Asterisks in Python

```{code-cell} ipython3
---
id: M7hIUZyN8CE_
slideshow:
  slide_type: '-'
---
3 * 5
```

```{code-cell} ipython3
---
id: 9xoOh5PL8FB9
slideshow:
  slide_type: fragment
---
3 ** 5
```

```{code-cell} ipython3
---
id: _q6Xl8NJ8HA_
slideshow:
  slide_type: slide
---
# Using * to unpack iterables into a list/tuple
numbers = [2, 1, 3, 4, 7]
more_numbers = [*numbers, 11, 18]
print(*more_numbers, sep=', ')

first, *rest = [*numbers, 11, 18]
print(rest, sep=', ')
```

```{code-cell} ipython3
---
id: ZHh2C4uH9TDl
slideshow:
  slide_type: slide
---
# Unpack a dictionary
date_info = {'year': "2020", 'month': "01", 'day': "01"}
filename = "{year}-{month}-{day}.txt".format(**date_info)
print(filename)
```

+++ {"id": "XN_p33qL97B9", "slideshow": {"slide_type": "slide"}}

### zip

+++ {"id": "0xrey4AK-CNJ", "slideshow": {"slide_type": "-"}}

The zip() function takes iterables (can be zero or more), aggregates them in a tuple, and returns it.

```{code-cell} ipython3
---
id: vv1tawNe98k3
slideshow:
  slide_type: fragment
---
languages = ['Java', 'Python', 'JavaScript']
versions = [14, 3, 6]

result = list(zip(languages, versions))
print(result)
```

+++ {"slideshow": {"slide_type": "slide"}}

`zip` is almost the inverse of itself. It result in the same lists that you started with.

```{code-cell} ipython3
---
slideshow:
  slide_type: '-'
---
list(zip(*result))
```

```{code-cell} ipython3
---
slideshow:
  slide_type: slide
---
mat = [[1, 2, 3],
        [4, 5, 6]]
mat_transpose = list(zip(*mat))
print(mat_transpose)
print(list(zip(*mat_transpose)))
```

```{code-cell} ipython3
---
id: Y0paQcOzAf6E
slideshow:
  slide_type: slide
---
for (x, y) in zip(languages, versions): # pairs of items pulled from  two lists
  print(x, y)
```

```{code-cell} ipython3
---
id: cD5QKJ3rBVn5
slideshow:
  slide_type: slide
---
keys = ['spam', 'eggs', 'toast']
vals = [1, 3, 5]

D2 = {}
for (k, v) in zip(keys, vals):
  D2[k] = v

D2
```

+++ {"id": "hJL1tFuoBH3N", "slideshow": {"slide_type": "slide"}}

zip function is more general than this example suggests. For instance,
it accepts any type of sequence (really, any iterable object, including files), and
it accepts more than two arguments. With three arguments, as in the following example,
it builds a list of three-item tuples with items from each sequence, essentially projecting
by columns (technically, we get an N-ary tuple for N arguments):

```{code-cell} ipython3
:id: igHuGfuVA4cF

T1, T2, T3 = (1,2,3), (4,5,6), (7,8,9)
list(zip(T1, T2, T3))
```

+++ {"id": "6HQY3GPU-S3H"}

The * operator can be used in conjunction with zip() to unzip the list.

```{code-cell} ipython3
:id: miGfL6H1-TZp

coordinate = ['x', 'y', 'z']
value = [3, 4, 5]

result = zip(coordinate, value)
result_list = list(result)
print(result_list)

c, v =  zip(*result_list)
print('c =', c)
print('v =', v)
```

+++ {"id": "ObA9PRtQL9hT", "slideshow": {"slide_type": "slide"}}

### Classes

+++ {"id": "hAzL_lTkL9hU", "slideshow": {"slide_type": "-"}}

#### Defining a Class

The syntax for defining classes in Python is :

```{raw-cell}
int length(struct date) {
    return // convert date into ssome form of legnth
}
struct date {
     int year;
     int month;
    (int) *length(struct date);
}
struct date dt;
dt->length()
```

```{code-cell} ipython3
---
id: F0xt8IhX0DN8
slideshow:
  slide_type: slide
---
class Addition:
    # first = 0
    # second = 0
    answer = 0 # Class member

    # parameterized constructor
    def __init__(self, first, second):
        self.first = first
        self.second = second

    def calculate(self):
        self.answer = self.first + self.second

# creating object of the class
# this will invoke parameterized constructor
Add = Addition
obj = Add(1000, 2000)

# closures (inside out classes where functionwraps the class data)
def counter(count_init):
    counter = count_init
    def increment(inc):
        nonlocal counter
        counter = counter + inc
        return counter
    return increment

c_inc = counter(0)
c_inc(5)
c_inc(3)
```

```{code-cell} ipython3
---
id: RWdbaGigL9hU
slideshow:
  slide_type: slide
---
class Person:

    # Constructor
    def __init__(self, name):
        self.name = name  # Create an instance variable

    # Instance method
    def getName(self):
        return self.name

    # To check if this person is an employee
    def isEmployee(self):
        return False

# Making an instance from a class
g = Person('Fred')  # Construct an instance of the Person class

# Accessing attributes
print(g.name)

# Calling methods
print(g.getName())
```

+++ {"id": "dqdK--1VtSo8", "slideshow": {"slide_type": "slide"}}

* The **self** parameter is required in the method definition, and it must come first before the other parameters. It must be included in the definition because when Python calls this method later (to create an instance), the method call will automatically pass the self argument.
* Every method call associated with an instance automatically passes **self**, which is a reference to the instance itself; it gives the individual instance access to the attributes and methods in the class.

+++ {"id": "8w7Os14f0qv3", "slideshow": {"slide_type": "slide"}}

In a class, the implicit behavior of passing the object as the first argument is avoided if a method is declared as static, as shown in the following example.

```{code-cell} ipython3
---
id: CJc8OOIR0Uif
slideshow:
  slide_type: slide
---
class A(object):

    @staticmethod
    def stat_meth():
        print("Look no self was passed")

a = A()
a.stat_meth()
```

+++ {"id": "zmSZEpuUwZPm", "slideshow": {"slide_type": "slide"}}

#### Inheritance

Inheritance is the capability of one class to derive or inherit the properties from another class. The benefits of inheritance are:

* It represents real-world relationships well.
* It provides reusability of a code. We don’t have to write the same code again and again. Also, it allows us to add more features to a class without modifying it.
* It is transitive in nature, which means that if class B inherits from another class A, then all the subclasses of B would automatically inherit from class A.

```{code-cell} ipython3
---
id: 8qnI09X2vpw8
slideshow:
  slide_type: slide
---
# Inherited or Subclass (Note Person in bracket)
class Employee(Person):

    # Constructor
    def __init__(self, name, year):
        self.years = year  # Create an instance variable
        super().__init__(name)

    # Overriding Methods from the Parent Class
    def isEmployee(self):
        return True

# Driver code
emp = Person("Geek1")  # An Object of Person
print(emp.getName(), emp.isEmployee())

emp = Employee("Geek2", 20) # An Object of Employee
print(emp.getName(), emp.isEmployee())
```

+++ {"id": "BUUw6EO1Y_zG", "slideshow": {"slide_type": "slide"}}

### Iterators

+++ {"id": "sJRb8KpFY_zI", "slideshow": {"slide_type": "-"}}

#### An iterator is an object that can be iterated upon, meaning that you can traverse through all the values.

```{code-cell} ipython3
---
id: lv2d_g5tY_zJ
slideshow:
  slide_type: fragment
---
mytuple = ("apple", "banana", "cherry")
myit = iter(mytuple) # define

print(next(myit)) # apple
print(next(myit)) # banana
print(next(myit)) # cherry
# Try this (uncomment)
# print(next(myit)) # raises StopIteration exception (not all exceptions are errors)
```

+++ {"id": "E9gzSlmbY_zP", "slideshow": {"slide_type": "slide"}}

#### Strings, lists, tuples, dictionaries, and sets are all iterable objects.

```{code-cell} ipython3
---
id: 85n-oj49Y_zQ
slideshow:
  slide_type: '-'
---
mystr = "banana"
myit = iter(mystr)

print(next(myit))
print(next(myit))
print(next(myit))
print(next(myit))
print(next(myit))
print(next(myit))
```

+++ {"id": "eYfmOJZEY_zU"}

#### Looping Through an Iterator

+++ {"id": "Kj14kEvZY_zU"}

C-style approach

```{code-cell} ipython3
:id: 6gnYMQG4Y_zV

mytuple = ("apple", "banana", "cherry")
i = 0
while (i < len(mytuple)):
    print(mytuple[i])
    i += 1
```

+++ {"id": "FYkBGJ37Y_zZ", "slideshow": {"slide_type": "slide"}}

A better approach

```{code-cell} ipython3
---
id: shCmE0HNY_zb
slideshow:
  slide_type: '-'
---
mytuple = ("apple", "banana", "cherry")

print(mytuple)

for idx, val in enumerate(mytuple):
  print(idx, val)
```

```{code-cell} ipython3
---
id: h-rgh8adL8LK
slideshow:
  slide_type: slide
---
mytuple = ("apple", "banana", "cherry")

print(mytuple)

for val in mytuple:
  print(val)
```

```{code-cell} ipython3
:id: dUITksMsY_ze

mystr = "banana"

for x in mystr:
  print(x)
```

```{code-cell} ipython3
:id: LLO7jfIIY_zh

# Iterating over dictionary
d = dict()
d['xyz'] = 123
d['abc'] = 345
for i in d :
    # print("%s  %d" %(i, d[i]))
    print("{}\t{}".format(i, d[i]))
```

+++ {"id": "6tDkn1koY_zj", "slideshow": {"slide_type": "slide"}}

#### Create an Iterator

+++ {"id": "jHEapd37Y_zk", "slideshow": {"slide_type": "-"}}

To create an object/class as an iterator, you have to implement the methods \_\_iter\_\_() and \_\_next\_\_() to your object.


*   **\_\_iter\_\_** method that is called on initialization of an iterator. This should return an object that has a \_\_next\_\_ method.
*   **\_\_next\_\_** should return the next value for the iterable. This method should raise a **StopIteration** to signal the end of the iteration.

```{code-cell} ipython3
---
id: 60grpN8LY_zk
slideshow:
  slide_type: slide
---
class MyNumbers:
  def __iter__(self):
    self.a = 1
    return self

  def __next__(self):
    x = self.a
    self.a += 1
    return x

myinstance = MyNumbers()
myiter = iter(myinstance)

print(next(myiter))
print(next(myiter))
print(next(myiter))
print(next(myiter))
print(next(myiter))
```

+++ {"id": "MJZGzf9IY_zn", "slideshow": {"slide_type": "slide"}}

Stop after 10 iterations by raising the StopIteration exception

```{code-cell} ipython3
---
id: xkWpi_H_Y_zo
slideshow:
  slide_type: '-'
---
class MyNumbers:
  def __iter__(self):
    self.a = 1
    return self

  def __next__(self):
    if self.a <= 10:
      x = self.a
      self.a += 1
      return x
    else:
      raise StopIteration

myclass = MyNumbers()
myiter = iter(myclass)
thing = ["Apple", "Audi", "Pasta", "dog", "UMAINE"]
category = ["fruit", "car",  "food", "animal"]

zipped = zip(thing, category)
zipped_l = list(zipped)
print(zipped_l)
print(list(zip(zipped_l[0], zipped_l[1], zipped_l[2], zipped_l[3])))
```

+++ {"id": "yEGblopEY_zq"}

#### zip function
Combine mulitple iterator

```{code-cell} ipython3
:id: UsumWzQJY_zr

# Two separate lists
thing = ["Apple", "Audi", "Pasta", "dog", "UMAINE"]
category = ["fruit", "car",  "food", "animal"]

# Combining lists and printing
for t, c in zip(thing, category):
    print("{} is {}".format(t, c))
```

+++ {"id": "8FN4iCf3Y_zx"}

Use "*" operator to unzip

```{code-cell} ipython3
:id: QI75XXjIY_zx

l1,l2 = zip(*[('Apple', 'fruit'),
              ('Audi', 'car'),
              ('Pasta', 'food')
           ])

# Printing unzipped lists
print(l1)
print(l2)
```

+++ {"id": "6wqnuO2uY_z1", "slideshow": {"slide_type": "slide"}}

## yield vs return

+++ {"id": "kcQwfXs3_JA_", "slideshow": {"slide_type": "-"}}

Unlike normal functions that return a value and exit, **generator functions** automatically
suspend and resume their execution and state around the point of value generation.
Because of that, they are often a useful alternative to both computing an entire series
of values up front and manually saving and restoring state in classes. Because the state
that generator functions retain when they are suspended includes their entire local
scope, their local variables retain information and make it available when the functions
are resumed.


The chief code difference between generator and normal functions is that a generator
**yields** a value, rather than returning one—the yield statement suspends the function
and sends a value back to the caller, but retains enough state to enable the function to
resume from where it left off. When resumed, the function continues execution immediately
after the last yield run. From the function’s perspective, this allows its code
to produce a series of values over time, rather than computing them all at once and
sending them back in something like a list.

```{code-cell} ipython3
---
id: dcU7dFMnY_z2
slideshow:
  slide_type: slide
---
def simpleGeneratorFun():
    x = 5
    # print(x)
    yield 1 + x

    x = 2*x
    #print(x)
    yield 1 + x


    x = 2*x
    #print(x)
    yield 1 + x
```

```{code-cell} ipython3
---
id: mxrp3zwE-nmI
slideshow:
  slide_type: '-'
---
simpleGeneratorFun()
```

+++ {"id": "j0yItSVhAF3j", "slideshow": {"slide_type": "slide"}}

the next(X) built-in calls an object’s
`X.__next__()` method for us:

```{code-cell} ipython3
---
id: ORAXLOxG_sOR
slideshow:
  slide_type: '-'
---
x = simpleGeneratorFun()
next(x)
```

```{code-cell} ipython3
---
id: aY3m0CwoN0ym
slideshow:
  slide_type: fragment
---
list(simpleGeneratorFun())
```

```{code-cell} ipython3
---
id: QLV9WZDSNyh6
slideshow:
  slide_type: skip
---
# Driver code to check above generator function
# simpleGeneratorFun()
list(simpleGeneratorFun())

#for value in simpleGeneratorFun():
#    print(value)
```

+++ {"id": "gHaXG1EHY_z5", "slideshow": {"slide_type": "slide"}}

Yield are used in Python generators. A generator function is defined like a normal function, but whenever it needs to generate a value, it does so with the yield keyword rather than return. If the body of a def contains yield, the function automatically becomes a generator function.

```{code-cell} ipython3
---
id: C7j8oMVIY_z5
slideshow:
  slide_type: slide
---
def nextSquare():
    i = 1;

    # An Infinite loop to generate squares
    while True:
        yield i*i
        i += 1  # Next execution resumes
                # from this point

# Driver code to test above generator
# function
for num in nextSquare():
    if num > 100:
         break
    print(num)
```

+++ {"id": "a60k2X4QY_z8", "slideshow": {"slide_type": "slide"}}

## Modules and Packages

```{code-cell} ipython3
---
id: D0szgI5ZY_z_
slideshow:
  slide_type: '-'
---
# Packages are stored on PyPi servers by default and can be directly installed from github as well.
!pip install import-ipynb
```

```{code-cell} ipython3
---
id: 7U66Oi23Y_0D
slideshow:
  slide_type: '-'
---
import import_ipynb
```

```{code-cell} ipython3
---
slideshow:
  slide_type: slide
---
%%writefile myModule.py

s = "hi from my module"
```

+++ {"slideshow": {"slide_type": "-"}}

Every python file is a module and can be imported. import executes everything in the module.

```{code-cell} ipython3
---
nbgrader:
  cell_type: code
  checksum: ed3306295ddc8cae2191697d284965a8
  grade: true
  grade_id: cell-5b99e5e743718896
  locked: false
  points: 10
  schema_version: 3
  solution: true
  task: false
slideshow:
  slide_type: fragment
---
import myModule
```

```{code-cell} ipython3
---
id: W3v5ddVBY_0J
slideshow:
  slide_type: slide
---
import sys
import os

dirpath = os.getcwd()
print("current directory is : " + dirpath)
foldername = os.path.basename(dirpath)
print("Directory name is : " + foldername)
```

+++ {"slideshow": {"slide_type": "slide"}}

You can create nested modules by directory structure:

```{code-cell} ipython3
---
slideshow:
  slide_type: '-'
---
!mkdir -p my_module
```

+++ {"slideshow": {"slide_type": "-"}}

You can not import an empty directory. You have to drop (maybe empty) `__init__.py` in the directory.

```{code-cell} ipython3
---
slideshow:
  slide_type: fragment
---
%%writefile my_module/__init__.py
""" My module is the best module ever. """
def identity(x):
    """ This function always returns what it is given """
    return x
```

+++ {"slideshow": {"slide_type": "slide"}}

Now you can import my_module.

```{code-cell} ipython3
---
slideshow:
  slide_type: '-'
---
import my_module
help(my_module)
```

+++ {"slideshow": {"slide_type": "slide"}}

You can move python files into hierarchies and directories.

```{code-cell} ipython3
---
slideshow:
  slide_type: '-'
---
!cp myModule.py my_module
```

```{code-cell} ipython3
---
slideshow:
  slide_type: '-'
---
import my_module.myModule as mmmm
mmmm.s
```

+++ {"slideshow": {"slide_type": "slide"}}

## Read More

1. [List of all Python Keywords](https://docs.python.org/3/reference/lexical_analysis.html#keywords)

    False      await      else       import     pass
    None       break      except     in         raise
    True       class      finally    is         return
    and        continue   for        lambda     try
    as         def        from       nonlocal   while
    assert     del        global     not        with
    async      elif       if         or         yield



2. [List of all Python builtins](https://docs.python.org/3/library/functions.html#built-in-funcs)

3. [List of all Python operators and their precedence](https://docs.python.org/3/reference/expressions.html?highlight=bitwise%20operators#operator-precedence)

+++ {"deletable": false, "editable": false}

## Submission

Make sure you have run all cells in your notebook in order before running the cell below, so that all images/graphs appear in the output. The cell below will generate a zip file for you to submit. **Please save before exporting!**

These are some submission instructions.

```{code-cell} ipython3
:deletable: false
:editable: false

# Save your notebook first, then run this cell to export your submission.
grader.export(run_tests=True)
```
