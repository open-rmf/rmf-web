// import { Task } from 'api-client';
import { UserProfile } from './authenticator';

export enum RmfAction {
  TaskRead = 'task_read',
  TaskSubmit = 'task_submit',
  TaskCancel = 'task_cancel',
}

export function getActionText(action: string): string {
  return action.replace(/(?:^|_)([a-z])/g, (_, p1: string) => ` ${p1.toUpperCase()}`).slice(1);
}

export class Enforcer {
  static canCancelTask(_profile: UserProfile): boolean {
    // if (profile.user.is_admin) {
    //   return true;
    // }
    // // for (const p of profile.permissions) {
    // //   if (p.authz_grp === task.authz_grp && p.action === RmfAction.TaskCancel) {
    // //     return true;
    // //   }
    // // }
    // return false;

    // FIXME: Allow all users to cancel for now
    return true;
  }
}
