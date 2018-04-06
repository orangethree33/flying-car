#Project: 3D Motion Planning
---


##Required Steps of this README
###1.Provide a Writeup /README which includes the rubric points .

###2.Exlaxin the Starter Code.  Test the `motion_planning,py` and discribe how the scripts works

###3.Implement my path planning algorithm.

---

## Writeup / README

1. You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

##Explain the Starter Code

 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
  First let me explain the `plannin_utils.py`,  that add some functions like  create_ grid ,in order to return a gird representation of a 2D configuration space based on given obstacle data, and define the  altitude of the drone and the safety distance, also use a function do guide the drone to take actions .the use A* ,the search algorithm to perform a search.
 `motion_planning.py`  it is basically a modified version of ` backyard_flyer.py` ,which has same functions such as arming transition ,takeoff transition, and some callbacks functions ,these functions  we had used in the backyard flyer project, so it is not very hard to  understand  ,but it also add some new functions  and classes to perform the state of the drone and the action that our drone to do next,
and use a  collinearity test to remove some unnecessary points.

##Implement my path
 1.I modify the code to read the global home location from the first line the `colliders.csv` flie, I check the starter code and I change the goal position as -m north and -m east of map center, also it is can be changed.

2.About the a star , I add the diagonal  motions to it and assign them a cost of sqrt(2).

##Executing the flight 
 I will the add the pictures of the flying situations to the zip because there is some pictures problems I can not deal with about this markdown editor. hope you understand.












