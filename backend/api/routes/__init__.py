# backend/api/routes/__init__.py
from fastapi import APIRouter

from .status import router as status_router
from .pose import router as pose_router
from .map import router as map_router
from .costmap import router as costmap_router
from .scan import router as scan_router
from .bag import router as bag_router
from .plan import router as plan_router

router = APIRouter(prefix="/api/v1")

router.include_router(status_router)
router.include_router(pose_router)
router.include_router(map_router)
router.include_router(costmap_router)
router.include_router(scan_router)
router.include_router(bag_router)
router.include_router(plan_router)
