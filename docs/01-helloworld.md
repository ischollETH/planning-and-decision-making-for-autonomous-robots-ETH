# Hello world :computer:

<table>
  <tr>
    <th><i>Prerequisites:</i></th><td><a href="./00-preliminaries.html" target="_top">Preliminaries</a></td>
  </tr>
</table>

In this tutorial we will test that you have all the tools set up correctly.

### Fork the repository

Let's start by forking the [exercise repository](https://github.com/idsc-frazzoli/PDM4AR-exercises) on your private
account (top right button). If the repository is not visible to you make sure you had sent us your GitHub name via
the [form](https://forms.gle/U89L1rgWeF8GUJz36), we will give you access as soon as possible.
The fork operation will create your personal copy of the repository containing the exercises.

#### Privacy

After forking the repo, you need to tweak some privacy setting such that other students cannot see your solutions. On
your forked repo, go to `Settings`, `Manage access` and feel free to remove everyone, in particular
the `PDM4AR-students` group.

![image](https://user-images.githubusercontent.com/18750753/133974277-518557f4-7727-42e9-84fc-ead9be212b37.png)

### Running the test "exercise"

We are finally ready to start, clone the newly **forked** repository on your local computer
(if your GitHub name is `YOURNAME` and you have [ssh enabled](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)
this can be done with `git clone git@github.com:YOURNAME/PDM4AR-exercises.git`, other methods are available as well, googling is your friend).

Enter the folder that just got cloned with

```shell
cd PDM4AR-exercises
```

Now let's build the docker image

```shell
make build
```

this will take a few seconds. If the image was build correctly now we can test running the container

```shell
make run-test
```

at the end of the execution a new folder `out-docker` should appear. The folder should have the following structure:

![image](https://user-images.githubusercontent.com/18750753/133898341-cf56b7ad-6c4a-40ba-9860-885474e32c73.png)

Open the `report.html` in your favorite browser to see the results of your test.


#### Setting up the upstream branch for updates

From time to time we might release updates or new exercises on the "parent" repository (the one you forked from).
In order to simply get the updates in the future we need to tell git that there exist an _upstream_ repository to pull from.
You can set this running `make set-upstream`.

Verify that everything went well typing `git remote -v` in your terminal. You should get something like this (`alezana` -> `YOUR GITHUB USERNAME`):
![image](https://user-images.githubusercontent.com/18750753/137486162-4aa48862-0c52-4e7d-987f-6d4b57582ad1.png)

# Exercise1 - Lexicographic comparison

Let's now try to solve our first exercise! It will be a very simple one.

First, open the source folder (`src`), it should have this form:
![Screenshot from 2021-09-21 10-06-31](https://user-images.githubusercontent.com/18750753/134135276-9676f025-6aee-43f8-9a6e-37737346414c.png)

The exercises that you need to modify are inside the `exercises` folder. Do **not modify** any other file which is not
inside `exercises`, it might result in breaking everything. Now open the file `exercises/ex01/ex1.py` and try to
implement a function that compares two vectors according to a lexicographic order. For each entry, lower is better.

Note that the function is _type annotated_:
![Screenshot from 2021-09-21 10-12-57](https://user-images.githubusercontent.com/18750753/134135930-884af68d-f5d9-4a00-b06f-f911468c400b.png)
Despite Python is not a strongly typed language as C++ or similar, python annotations are a great way to develop with
clear interfaces. Learn more [here](https://www.python.org/dev/peps/pep-0484/).

Something like this

```python
def compare(a: Tuple[float], b: Tuple[float]) -> ComparisonOutcome:
```

reads as follows:
> _"compare is a function that takes two arguments, each is expected to be a tuple of floats. The type of the returned argument should be ComparisonOutcome"_

You are now ready to implement your solution and check it with

```shell
make build run-exercise1
```

Open the corresponding report and check your results!

### Notes on the development environment

If you decided to go with PyCharm you can open the cloned repository as a project inside it. Few steps that ease the
development:

* Mark the source folder (`src`) as "Source folder". Right click on it then "Mark Directory as".
* If you want to be able to debug in an interactive way with breakpoints and similar, you need to install python3.8 or
  superior natively on your laptop and the required packages in the `requirements.txt` via pip. Then you can debug the
  exercises running the `app_main.py`
