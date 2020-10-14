import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

enum taskState {
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

  static formatStatus(status: string): string[] {
    return status.split('|');
  }

  static getStateLabel(state: number): string {
    switch (state) {
      case taskState.STATE_QUEUED:
        return 'QUEUED';
      case taskState.STATE_ACTIVE:
        return 'ACTIVE';
      case taskState.STATE_COMPLETED:
        return 'COMPLETED';
      case taskState.STATE_FAILED:
        return 'FAILED';
      default:
        return 'UNKNOWN';
    }
  }

  private _tasks: Record<string, RomiCore.TaskSummary> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
}
