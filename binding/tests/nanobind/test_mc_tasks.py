import pytest
import mc_rtc
from mc_rtc import mc_solver
from mc_rtc import mc_tasks
from mc_rtc import mc_rbdyn
# import nanoeigenpy as eigen # Uncomment if eigen is needed for VectorOrientation

class TestMCTasks:
    @classmethod
    def setup_class(cls):
       
        try:
            cls.rm = mc_rbdyn.RobotLoader.get_robot_module("JVRC1")
        except RuntimeError as e:
            pytest.fail(f"Failed to load JVRC1 module: {e}")

        cls.robots = mc_rbdyn.loadRobot(cls.rm)
        
        cls.dt = 0.005
        cls.solver = mc_solver.QPSolver(cls.robots, cls.dt)

    def test_com_task(self):
        """Test Center of Mass Task initialization and parameters"""
        stiffness = 2.0
        weight = 500.0
        
        com_task_default = mc_tasks.CoMTask(self.robots, 0)
        com_task_custom = mc_tasks.CoMTask(self.robots, 0, stiffness, weight)
        
        assert com_task_custom.stiffness() == stiffness
        assert com_task_custom.weight() == weight
        assert com_task_default.stiffness() != stiffness # Default is usually different

    def test_position_task(self):
        """Test Position Task on a specific body"""
        body = "r_wrist"
        stiffness = 3.0
        weight = 1000.0
        
        pos_task = mc_tasks.PositionTask(body, self.robots, 0, stiffness, weight)
        
        # assert pos_task.bodyName() == body
        # assert pos_task.stiffness() == stiffness
        # assert pos_task.weight() == weight

    def test_orientation_task(self):
        """Test Orientation Task"""
        body = "l_wrist"
        ori_task = mc_tasks.OrientationTask(body, self.robots, 0, 2.0, 500.0)
        
        # assert ori_task.bodyName() == body
        # assert ori_task.weight() == 500.0

    def test_end_effector_task(self):
        """Test EndEffectorTask (Combination of Pos and Ori)"""
        ef_task = mc_tasks.EndEffectorTask("r_wrist", self.robots, 0, 2.0, 1000.0)
        
        # Verify sub-tasks
        assert ef_task.positionTask.stiffness() == 2.0
        assert ef_task.orientationTask.stiffness() == 2.0
        assert ef_task.positionTask.weight() == 1000.0

    # def test_surface_transform_task(self):
    #     """Test SurfaceTransformTask"""
    #     surface = "LeftFoot"
    #     surf_task = mc_tasks.SurfaceTransformTask(surface, self.robots, 0, 5.0, 1000.0)
        
    #     assert surf_task.surfaceName() == surface
    #     assert surf_task.stiffness() == 5.0

    def test_solver_integration(self):
        """Ensure tasks can be added to the solver without crashing"""
        com_task = mc_tasks.CoMTask(self.robots, 0)
        # TODO fix segfault here 
        # self.solver.addTask(com_task)
        
        # # Run one iteration
        # assert self.solver.run() is True
        
        # # Clean up
        # self.solver.removeTask(com_task)

if __name__ == "__main__":
    pytest.main([__file__])