import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export enum TaskState {
  STATE_QUEUED = 0,
  STATE_ACTIVE,
  STATE_COMPLETED,
  STATE_FAILED,
}

export default class TaskManager extends EventEmitter<Events> {
  tasks(): RomiCore.TaskSummary[] {
    console.log(this._tasks);
    return Array.from(Object.values(this._tasks));
  }

  startSubscription(transport: RomiCore.Transport) {
    this._subscriptions.push(
      transport.subscribe(RomiCore.taskSummaries, (taskSummary) => {
        this._tasks[taskSummary.task_id] = taskSummary;
        this.emit('updated');
      }),
    );
  }

  stopAllSubscriptions(): void {
    this._subscriptions.forEach((sub) => sub.unsubscribe());
  }

  static formatStatus(status: string) {
    const statusSplited = status.split('|');
    const action = statusSplited[0];
    const remainingPhases1 = statusSplited[1];
    const remainingPhases2 = statusSplited[2];
    const statusLabel = statusSplited[0].split(':')[0];
    return { action, remainingPhases1, remainingPhases2, statusLabel };
  }

  static getStateLabel(state: number): string {
    switch (state) {
      case TaskState.STATE_QUEUED:
        return 'QUEUED';
      case TaskState.STATE_ACTIVE:
        return 'ACTIVE';
      case TaskState.STATE_COMPLETED:
        return 'COMPLETED';
      case TaskState.STATE_FAILED:
        return 'FAILED';
      default:
        return 'UNKNOWN';
    }
  }

  private _tasks: Record<string, RomiCore.TaskSummary> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
}
