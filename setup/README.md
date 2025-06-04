# Setup and Requirements

This document includes the necessary tools used throughout the course.  
You don't need to fully understand what each one does, how the underlying systems work, or even follow every tutorial right now. Just make sure you install them, try the exercises, or at least feel confident that you could.



---

## How to Approach This Document

The ultimate goal at this stage is to:

- Install **Git**
- Install **VSCode**
- Install **Docker**
- Set up a **devcontainer**
- Install **ROS2** inside that devcontainer

How you choose to get it done is no concern of mine, so just treat this document as a mere set of suggestions, not strict instructions.

You can probably get everything set up in under an hour or two, but take a moment to read some of the text along the way. It's good for your brain and you'll pick up some random knwledge and come out a slightly more informed human.


## Tools
### Git  
A version control system so you can keep track of the history of the code you write. [What is Git?](https://git-scm.com/book/en/v2/Getting-Started-What-is-Git)

**Tutorials:**  
- If you add together the number of tutorials for all programming languages, there are probably still 25% more Git tutorials.  
- Recommended:  
  + Tutorial from Kaveh's Robotic Mapping course (recommended): [Markdown](./Github.md) [Word](Github%20Guide.docx)
  + OG Git tutorial: [Git SCM](https://git-scm.com/docs/gittutorial)

**Exercise:**  
- Install [Git](https://git-scm.com/downloads) based on your OS.  
- Clone this repo and open it locally and continue reading it locally. That's it.
- Bonus point (very optional): There are 5 spelling errors in this document. If you find any, make a pull request on GitHub.

---
### VS Code  
An editor for writing text that has meaning, or so we think.
- Yes, it's from evil Microsoft. And fun fact: the version you download isn't actually fully open source.
- You can use other editors, but VS Code works very smothly with Docker and devcontainers. So... just use it.

**Exercise:**  
- Install it [Download](https://code.visualstudio.com/)
- Get comfortable creating files, editing them, opening the terminal, etc.

---

### Docker / Devcontainer  

> This course heavily relies on the ROS 2 ecosystem, which is best supported on Ubuntu.  
> If you already have an Ubuntu system, you can skip the rest of this setup (although it's not recommended). If not, follow the instructions to the end. 

Since ROS 2 works best on Ubuntu, we want to simulate that environment on your OS.  
You can use a virtual machine (VM) or Docker, the lightweight alternative. It gives your programs the illusion they're running on another OS.


You may ask, please, someone explain Docker to me like I am an idiot. [Sure, I'd say](https://www.reddit.com/r/docker/comments/keq9el/please_someone_explain_docker_to_me_like_i_am_an/)

Or you may ask How Docker Works? [Huh I'd say](https://codeahoy.com/2019/04/12/what-are-containers-a-simple-guide-to-containerization-and-how-docker-works/)


Now, devcontainers are the integration of a Docker container with your editor (like VS Code). Sounds complicated, but you shouldn't care that much.

**Tutorials:**
- [How to set up a Devcontainer](https://github.com/ariarobotics/robotic-mapping/blob/main/resources/how%20to%20set%20up%20a%20devcontainer.md)

**Exercise:**  
- Install Docker based on your OS from the official website.  
- Follow the tutorial above or the official [documentation](https://code.visualstudio.com/docs/devcontainers/tutorial) to set up a devcontainer for python development. 
  You can `clone` this project to get a ready-made Ubuntu environment.


#### [GUI setup](https://github.com/ariarobotics/robotic-autonomy/tree/main/setup)


---

### Linux  
Now I assume everyone has a devcontainer with Ubuntu set up.  
If so, Congratulazioni, You now hve a real OS instead of whatever your had before. Let's learn some Linux.

> [!NOTE]  
> You need to run all these command in the docker container you just made. [How?](https://stackoverflow.com/questions/69860182/how-to-detect-if-the-current-script-is-running-in-a-docker-build)

**Tutorials:**  
- [Ubuntu Command Line for Beginners](https://ubuntu.com/tutorials/command-line-for-beginners)  
- Or just Google it.

**Exercise:**  
- Then run some Linux commands:
  - Make a folder called `disaster_drill` with a subfolder named after your favorite *Chernobyl* character (e.g. `legasov`).
  - Change directory into that subfolder.
  - Use `fortune` with `tee` to generate a quote, display it, and write it into `quote.txt`.
  - Add a separator (`---`), then append the curent date using `date >>`, and your username and hostname using `echo $USER@$HOSTNAME >>`.
  - Go up one directory.
  - Compress the folder using `tar -czf`.
  - Create a hash using `sha256sum` and save it to `hash.txt`.
  - Move both `meltdown.tar.gz` and `hash.txt` to the parent folder using `mv`.
  - Go back to the root with `cd ../`.
  - Do a 5-second countdown using a `for` loop and `sleep 1`.
  - Finally, delete the whole `disaster_drill` folder using `rm -rf`.

- First, install Python using `apt install` command
  - Write a Hello World python script and run it.
  - Run the [demo script](./DevContainer.md#test-python-script) 

> [!CAUTION]
> The answers are provided in `commands.sh.txt`
> You are allowed to use any kind of generative AI or just open the answers, but the purpose here is to learn some random commands. Try running them line by line to understand what each one does. Or, if you prefer, just look them up as needed.


---

<!-- 
### Python
You need to know this language. This course is not about a Job, is about you learning something, so if you don't, don't worry, you have the chance to learn it. But here, we just wanna install it. 

**Tutorials:**  
- [Python Intro (Google Drive)](https://drive.google.com/drive/folders/1xOLlGaIEghESVhHj1V0vGSbvy7_XGrYO?usp=drive_link)  
- [Real Python - Python 3 Introduction](https://realpython.com/learning-paths/python3-introduction/)

**Exercise:**   -->


### ROS 2  
The ROS 2 ecosystem is where most robotic algorithms live. Some are stable. Some are garbage (like its documentation).  

Okay here things get slightly more complicated. As I said, a docker container is almost an OS. So you could make a Ubuntu container and start installing everything you need for ROS 2 development. 

But even better, there is something called a Dockerfile, which is a recepie for setting up whatever you need for development. Meaning when the container is being made, everything is installed for you, like a full blown OS.


**Tutorial:**  
- [The second method (recommended)](./DevContainer.md)

**Exercise:**  
- Install ROS 2 in the container
- Run the turtlesim demo:
  ```bash
  ros2 run turtlesim turtlesim_node
  ros2 run turtlesim turtle_teleop_key
  ```
    - If the program raises an error about display connections, refer to [How to set up a Devcontainer](https://github.com/ariarobotics/robotic-mapping/blob/main/resources/how%20to%20set%20up%20a%20devcontainer.md) again. 
---

### Conclusion  
There you have it, you've successfully set up a Linux-powered environmment and are ready to start writing ROS packages.  
If you've finished everything above, you can now move on to doing actual stuff.
