from .models import User


class Enforcer:
    @staticmethod
    def _is_superadmin(user: User):
        return "_rmf_superadmin" in user.roles

    @staticmethod
    def can_submit_task(user: User):
        return Enforcer._is_superadmin(user) or "_rmf_task_submit" in user.roles
