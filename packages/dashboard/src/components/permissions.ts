import { Permission, Task, User } from 'api-client';

export class RmfAction {
  static TaskCancel = 'task_cancel';
}

export class Enforcer {
  static canCancelTask(user: User, permissions: Permission[], task: Task): boolean {
    if (user.is_admin) {
      return true;
    }
    for (const p of permissions) {
      if (p.authz_grp === task.authz_grp && p.action === 'task_cancel') {
        return true;
      }
    }
    return false;
  }
}
