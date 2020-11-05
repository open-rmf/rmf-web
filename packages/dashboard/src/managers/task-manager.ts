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

  private _tasks: Record<string, RomiCore.TaskSummary> = {};
  private _subscriptions: RomiCore.Subscription[] = [];
}
