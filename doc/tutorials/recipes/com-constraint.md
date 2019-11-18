---
layout: tutorials
---

In many cases, one may want to limit the accessible region of the Center of Mass. To do so, we will introduce a new constraint, `CoMIncPlaneConstr` that can be used to constrain the CoM to a convex region.

Let us present the API of this constraint:
```cpp
CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);
void set_planes(QPSolver & solver,
                const std::vector<mc_rbdyn::Plane> & planes,
                const std::vector<Eigen::Vector3d> & speeds = {},
                const std::vector<Eigen::Vector3d> & normalsDots = {},
                double iDist = 0.05,
                double sDist = 0.01,
                double damping = 0.1,
                double dampingOff = 0.);
```
This means that you can:
- Build a constraint applied on a given robot, with a given timestep.
- Set the planes of the constraint, i.e. update the actual region constraining the Center of Mass

How to "set planes"
---
First, a quick reminder on convex sets: they can be equivalently described by their vertices or by a set of linear inequalities.
That is to say, any point p inside a convex set can be expressed as the barycenter of the vertices, i.e. there exists a set of weights `\lambda_i`:
```
p = \sum_i \lambda_i v_i
\sum_i \lambda_i = 0
\forall i \lambda_i \geq 0
```
However, this is not very efficient for testing if a vertex is inside, as it requires basically solving a linear program. To actually test membership, the hyperplane representation is much better as it allows to test in a single matrix operation:
```
A p + b \geq 0
```
Where A is a matrix of stacked normals to each plane, and b is a vector of offsets.

Thus, to constrain the CoM to a static region, one only needs to construct a few normals and offsets, and put it in the constraint.

Example
---
Let's go back to the CoM controller we [constructed before]({{site.baseurl}}/tutorials/introduction/com-controller.html): the robot fell in a dynamic simulation because we asked it to go to far to the sides.
Let us now restrict its motion to a square, of sides 8cm. This can be expressed as:
```
-0.08 \leq x \leq 0.08
-0.08 \leq y \leq 0.08
```

Thus, in matrix form:
```
⎡ -1 0 0 ⎤ ⎡ x ⎤   ⎡ 0.08 ⎤
⎢ 1  0 0 ⎥ ⎢ y ⎥ + ⎢ 0.08 ⎥ \leq 0
⎢ 0 -1 0 ⎥ ⎣ z ⎦   ⎢ 0.08 ⎥
⎣ 0  1 0 ⎦         ⎣ 0.08 ⎦
```

We now only need to include this in the constraint:
```cpp
{% raw %}
//In the header:
# include <mc_solver/CoMIncPlaneConstr.h>

std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstraintPtr_;


//In the cpp, in the controller's initializer list:
comIncPlaneConstraintPtr_.reset(new mc_solver::CoMIncPlaneConstr(robots(), robots().robotIndex(), dt) );

//In the constructor itself:
    std::vector<mc_rbdyn::Plane> planes =
    {{{-1., 0., 0.}, 0.08},
     {{1., 0., 0.}, 0.08},
     {{0., -1., 0.}, 0.08},
     {{0., 1., 0.}, 0.08},
    };
 solver().addConstraintSet(*comIncPlaneConstraintPtr_);
 comIncPlaneConstraintPtr_->set_planes(solver(), planes);
{% endraw %}
```

Now if you launch the controller, you will see that the robot hits an invisible "wall", and is unable to accomplish the task.

Additional parameters
---

The additional parameters passed to `set_planes` fall in two categories:

- Dealing with moving planes
- Changing the interaction behaviour

For the first one, one can pass in the planes' speeds and plane normal's derivatives with respect to time as extra parameters to provide extra information.
Note that as when tracking a trajectory, the planes and their derivatives must be set at each timestep.

Then, one can tune the:

- Interaction distance: the planes will push away the CoM as soon as it gets closer to that distance.
- Safety distance: the distance from the CoM to the planes cannot get smaller than this.
- Damping and damping offset that govern how the repulsion behaves.
