import { Task } from 'api-client';
import { UserProfile } from './auth/contexts';

export class RmfAction {
  static TaskCancel = 'task_cancel';
}

export class Enforcer {
  static canCancelTask(profile: UserProfile, task: Task): boolean {
    if (profile.user.is_admin) {
      return true;
    }
    for (const p of profile.permissions) {
      if (p.authz_grp === task.authz_grp && p.action === 'task_cancel') {
        return true;
      }
    }
    return false;
  }
}
