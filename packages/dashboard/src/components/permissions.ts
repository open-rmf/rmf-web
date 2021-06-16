import { User } from '../../../rmf-auth/lib';

export class RmfRole {
  static SuperAdmin = '_rmf_superadmin';
  static TaskSubmit = '_rmf_task_submit';
  static TaskCancel = '_rmf_task_cancel';
  static TaskAdmin = '_rmf_task_admin';
}

export function canSubmitTask(user: User): boolean {
  return !!user.roles.find((r) => [RmfRole.SuperAdmin].includes(r));
}
