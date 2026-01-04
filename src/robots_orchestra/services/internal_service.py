from typing import Any, Dict, Optional

from robots_orchestra.services.base import ServiceBase

class InternalService(ServiceBase):
    def __init__(self):
        super().__init__("InternalService")