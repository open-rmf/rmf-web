import { User } from '../../../rmf-auth/lib';

export class RmfRole {
  static SuperAdmin = '_rmf_superadmin';
  static TaskSubmit = '_rmf_task_submit';
  static TaskCancel = '_rmf_task_cancel';
  static TaskAdmin = '_rmf_task_admin';
}

export class Enforcer {
  static canSubmitTask(user: User): boolean {
    return !!user.roles.find((r) => [RmfRole.SuperAdmin, RmfRole.TaskSubmit].includes(r));
  }

  static canCancelTask(user: User): boolean {
    return !!user.roles.find((r) => [RmfRole.SuperAdmin, RmfRole.TaskCancel].includes(r));
  }
}
