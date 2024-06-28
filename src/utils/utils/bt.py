from typing import Any
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import logging
from py_trees.composites import Sequence, Selector

class AreSampleLocationsGiven(Behaviour):
    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status

    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

class HasCamera(Behaviour):
    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status 
    
    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

class AreAllSamplesMeasured(Behaviour):
    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status
    
    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

class IsAtSampleLoaction(Behaviour):
    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status
    
    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

class LocateSamples(Behaviour):
    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status
    
    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

class Measure(Behaviour):
    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status
    
    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

class ApproachNextSample(Behaviour):


    def __init__(self, name: str, status: bool):
        super().__init__(name)
        self.s = status
    
    def setup(self, **kwargs: Any) -> None:
        self.logger.debug(f'Setup {self.name} node.')

    def initialise(self) -> None:
        self.logger.debug(f'Initialise {self.name} node.')
    
    def update(self) -> Status:
        self.logger.debug(f'Update {self.name} node.')
        return Status.SUCCESS if self.s else Status.FAILURE

def make_bt():
    are_locations_given = AreSampleLocationsGiven('are_locations_given', False)
    has_camera = HasCamera('has_camera', True)
    are_samples_measured = AreAllSamplesMeasured('are_samples_measured', True)
    is_at_sample = IsAtSampleLoaction('is_at_sample', True)
    locate_samples = LocateSamples('locate_samples', True)
    measure = Measure('measure', True)
    approach_sample = ApproachNextSample('approach_sample', True)

 
    locate_samples_seq = Sequence('locate_samples_seq', False, children=[has_camera, locate_samples])
    identify_samples = Selector('identify_samples', False, [are_locations_given, locate_samples_seq])
    measure_seq = Sequence('measure_seq', False, [is_at_sample, measure])
    measure_approach = Selector('measure_approach', False, [measure_seq, approach_sample])
    measure_all_samples = Selector('measure_all_samples', False, [are_samples_measured, measure_approach])
    main_task = Sequence('main_task', False, [identify_samples, measure_all_samples])

    return main_task

if __name__ == '__main__':
    logging.level = logging.Level.DEBUG
    bt = make_bt()
    bt.tick_once()