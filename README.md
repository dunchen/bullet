bullet
======

author: Dunchen

2014/7/11

Bullet-engine-shatter-plugin

###Summary
This is a bullet physics plugin, which adds the real time collision-and-shtter effect. It is very useful and impressive for its highly-effeciency compared with other professional special effect engine.


###What does it do
This plugin doing its task in the sub-tick-callback during the stimulation of the bullet engine. It will check every collision point between the rigidbodies and it will split the original rigibody and apply the force to the resulting shtters.

###The creative points
As we know, most of the shatter effects procided by the other engines are too time costing since they actually use many small nodes to represent the original rigidbody in order to get shatter effect. But this would make the question complicated since usually we don't want to spend too much time and effort on the construction of a wall, for example, in our FPS game desgin. But we want that wall in the game to be a little more real: it SHOULD collapse into shatters when we shoot RPG direct into the wall.
In order to achieve this aim, I write this prototype code based on the bullet engine, which doesn't provide a way to realize that.
I just first consider the situation of the world made of only box shapes rigidboies, which make the code much easier.
My code does its task in the sub-tick-callback during the stimulation of the bullet engine. It will check every collision point between the bullets and rigidbodies (the blocks) and it will split the original rigibody and apply the force to the resulting shtters to make them move in the way of being crushed by a bullet.


###Where needs improvement
I would to expand this prototype to include other collision shapes like sphere, compoundshape and triangle-mesh, but it was shown to be a very difficult task. In the next version, I would try to add the compound shape into the consideration.













//yeti comes down from the jokul~~
