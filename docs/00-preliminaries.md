# Preliminaries

- [Rules of the game](#rules-of-the-game)
- [Getting started](#getting-started)
    * [Git](#git)
        - [Installing Git](#installing-git)
        - [Creating a Github account](#creating-a-github-account)
        - [Resources](#-resources--)
        - [Goals](#--goals---)
    * [Docker](#docker)
        - [Resources](#-resources---1)
        - [Goals](#--goals----1)
    * [Makefiles](#makefiles)
    * [Python](#python)
        - [Resources](#-resources---2)
    * [IDEs (optional-recommended)](#ides---optional---recommended----)
- [Support](#support)

## Rules of the game

The programming exercises are not compulsory. Yet, they are a great way to understand and apply what you learn in class.

There will be two types of exercises:

- **Not-Graded** exercises;
- **Graded** ones.

The *Not-Graded* ones will be regularly released during the course.
These will NOT be graded, but they provide a playful way of studying.

There will be **1 Graded** exercise, featuring the same style of the not-graded ones but a bit more involved.
This can contribute up to 0.25 toward your final grade, only if they help to improve it. You will have 2/3 weeks to solve it:

| Graded Exercise    | **Release** date    | **Hand-in** date    |
|-----------------	|--------------	|--------------	|
| 1.                | End of Nov.   | Dec. 22nd (23:59 CET) |

You can solve the graded exercise in groups of up to 4 people. Detailed instructions on how to hand-in these exercises will
follow.

## Getting started

We will use:

- [Python](https://www.python.org/) as programming language;
- [Docker](https://www.docker.com/) as environment containerization;
- [Makefiles](https://en.wikipedia.org/wiki/Make_(software)) to launch the exercises.

If they all sound completely new to you do not panic. We will require a very basic use of each of them, but it is a good time
to start learning these tools since they are all widely adopted in modern robotics. If you get stuck in the process try
to pair up with some more experienced colleagues who can help you. If this still does not solve the problem, try to
reach out to the instructors on Moodle. If you are using Mac or Linux-based OS the process should be straight forward.
Windows can give some more hiccups, but it is supported as well.

### Git

Git is a version control software. Please find more explanation under the "Resources" paragraph if you have never heard of
it. You will need Git on your computer and a GitHub account.

Once you have them, send us your GitHub name via [this form](https://forms.gle/U89L1rgWeF8GUJz36), we will give you
access to the repository containing the exercises (this will take 1 day maximum).

#### Installing Git

Simply follow the steps for your OS at [this link](https://git-scm.com/downloads)

### Creating a GitHub account

If you do not have already a GitHub account create a new one [here](https://github.com/join)

#### _Resources_

- [Git introduction](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/preliminaries_git.html#sec:preliminaries-git)
- [Github tutorial](https://guides.github.com/activities/hello-world/)

#### **Checklist**

- [ ]  I have Git set up on my computer
- [ ]  I have a GitHub account
- [ ]  I know how to clone a repository
- [ ]  I have sent my GitHub name to the appropriate form

### Docker

We will run the exercises in a virtual environment (or better, in a container). Read the "Resources" section to get
a better understanding of it. Now let's install it on your computer:

* (Mac, Linux) [installation instructions](https://docs.docker.com/get-docker/)
* (Windows) the procedure is more complicated:
    + Follow the manual installation steps for Windows Subsystem for
      Linux [here](https://docs.microsoft.com/en-us/windows/wsl/install-win10). On step 1, follow the recommendation of
      updating to WSL 2. On step 6 you can download Ubuntu 18.04 LTS. You do not necessarily need to install Windows
      Terminal.
    + Now go [here](https://docs.docker.com/desktop/windows/install/) and follow the _Install Docker Desktop on Windows_
      instructions. You can then start Docker Desktop and follow the *quick start guide*.

#### _Resources_

- [Docker intro](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/preliminaries_docker_basics.html)

#### **Checklist**

- [ ] I have installed Docker on my computer
- [ ] I can run without errors the Docker hello-world (`docker run hello-world`)

### Makefiles

No action required if you are on Linux or Mac operating systems.

_For Windows_:
First you need to have followed the steps in the Docker section.

* Open an ubuntu terminal (search for *Ubuntu 18.04 LTS* in the search box on the bottom left of your screen and press
  enter) and run the following commands to install *make*:

Run update command to update package repositories and get latest package information.

```shell
sudo apt-get update -y
```

Run the install command with -y flag to quickly install the packages and dependencies.

```shell
sudo apt-get install -y make
```

### Python

Python will be the programming language adopted in this course.

#### _Resources_

- [Official documentation](https://docs.python.org/3/)
- [Python Tutorial](https://www.tutorialspoint.com/python/index.htm), in particular make sure to go through the basic
  data structures (tuples, lists, dictionaries, sets,...), loops (while, for,...), conditional statements (if-else),
  functions, classes and objects.

### IDEs (_optional-recommended_)

Using an [IDE](https://en.wikipedia.org/wiki/Integrated_development_environment) is completely optional and not
necessary.
But it provides a good set of tools that speed up the development (code navigation, debugging,...).

There are many good IDEs for python (Atom, Spyder, PyCharm,...), we recommend and will
support [PyCharm](https://www.jetbrains.com/pycharm/promo/?source=google&medium=cpc&campaign=14123077402&gclid=Cj0KCQjwv5uKBhD6ARIsAGv9a-xRdQ2ElK0YgSTtMWkKEfbR70PRdLthXgR2hhKxtYmgou2PVlJGRyQaAgr9EALw_wcB).

#### Debugging on Windows
Once again, this will require a little more effort.
To make everything work properly we will need to set up a "remote interpreter" in WSL.

You need two things:
- A **professional** version of PyCharm ([free for students](https://www.jetbrains.com/community/education/#students));
- Set up the remote interpreter via WSL: instruction [here](https://www.jetbrains.com/help/pycharm/using-wsl-as-a-remote-interpreter.html).

## Support

Use the forums on Moodle for general questions: this way, you can help other students who share your issues.
Open a [github issue](https://github.com/idsc-frazzoli/PDM4AR-exercises/issues) if you suspect that there is a bug in the code or in the instructions.
