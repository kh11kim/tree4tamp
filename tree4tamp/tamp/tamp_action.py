from dataclasses import dataclass, field
from typing import Optional, List

@dataclass
class TAMPAction:
    target_obj_name: str

@dataclass
class Attach(TAMPAction):
    parent_from: str
    parent_to: str
    name: Optional[str] = field(default_factory=lambda :None)

    # def reverse(self):
    #     raise NotImplementedError()
    
    @property
    def robot_name(self):
        if self.name == "pick":
            return self.parent_to
        elif self.name == "place":
            return self.parent_from
    
    def as_string(self):
        return f"{self.name}-{self.target_obj_name}-{self.parent_from}-{self.parent_to}"

@dataclass
class Pick(Attach):
    name: Optional[bool] = field(default_factory=lambda :"pick")

    # def reverse(self):
    #     return Place(
    #         obj_name=self.obj_name,
    #         parent_from=self.parent_to,
    #         parent_to=self.parent_from)

@dataclass
class Place(Attach):
    name: Optional[bool] = field(default_factory=lambda :"place")

    # def reverse(self):
    #     return Pick(
    #         obj_name=self.obj_name,
    #         parent_from=self.parent_to,
    #         parent_to=self.parent_from)