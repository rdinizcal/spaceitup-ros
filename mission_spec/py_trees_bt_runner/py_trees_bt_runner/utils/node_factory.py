from py_trees import composites, decorators, behaviours

from py_trees_bt_runner.behaviours.mars_rover_behaviours import *

NODE_FACTORY = {
    # Control nodes
    "Sequence": lambda name="Sequence": composites.Sequence(name=name, memory = False),
    "Fallback": lambda name="Fallback": composites.Selector(name=name, memory=False),
    "Inverter": lambda name="Inverter", child=None: decorators.Inverter(name, child),

    # Action nodes
    "NavigateToGoal": lambda name="NavigateToGoal", x="0", y="0": NavigateToGoal(name=name, x=x, y=y),
    "TakePicture": lambda name="TakePicture", **kwargs: TakePicture(name=name, **kwargs),

    # Condition nodes
    "IsPictureTaken": lambda name="IsPictureTaken", picture_id="0": behaviours.CheckBlackboardVariableExists(name=name, variable_name=f"picture_{picture_id}"),
    "LocationReached": lambda name="IsInLocation", x="0", y="0": LocationReached(name=name, x=x, y=y),
    "IsRobotClose": lambda name="IsRobotClose", **kwargs: IsRobotClose(name=name, **kwargs)
}