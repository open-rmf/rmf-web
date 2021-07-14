import { Task } from 'api-client';
import { User } from './auth/contexts';

export class RmfAction {
  static TaskCancel = 'task_cancel';
}

export class Enforcer {
  static canCancelTask(user: User, task: Task): boolean {
    if (user.profile.is_admin) {
      return true;
    }
    for (const p of user.permissions) {
      if (p.authz_grp === task.authz_grp && p.action === 'task_cancel') {
        return true;
      }
    }
    return false;
  }
}
