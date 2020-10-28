import * as RomiCore from '@osrf/romi-js-core-interfaces';
import EventEmitter from 'eventemitter3';

type Events = {
  updated: [];
};

export default class TaskManager extends EventEmitter<Events> {
  tasks(): RomiCore.TaskSummary[] {
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

  static getActorFromStatus(status: string) {
    // Gets the name of the robot if it has any
    // eslint-disable-next-line
    return status.match(/\[[A-Za-z]([a-zA-Z0-9\/]){3,}\]+/gi);
  }

  static formatStatus(status: string) {
    return status.split('|');
  }

  static getStateLabel(state: number): string {
    switch (state) {
      case RomiCore.TaskSummary.STATE_QUEUED:
        return 'QUEUED';
      case RomiCore.TaskSummary.STATE_ACTIVE:
        return 'ACTIVE';
      case RomiCore.TaskSummary.STATE_COMPLETED:
        return 'COMPLETED';
      case RomiCore.TaskSummary.STATE_FAILED:
        return 'FAILED';
      default:
        return 'UNKNOWN';
    }
  }

  private _tasks: Record<string, RomiCore.TaskSummary> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
}
