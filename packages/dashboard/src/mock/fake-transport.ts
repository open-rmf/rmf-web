import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import buildingMap from './data/building-map';
import fakeDispenserStates from './data/dispenser-states';
import fakeDoorStates from './data/door-states';
import fakeFleets from './data/fleets';
import fakeLiftStates from './data/lift-states';
import fakeTaskSummary from './data/task-summary';

const debug = Debug('FakeTransport');

export type DoorStateFactory = () => Record<string, RomiCore.DoorState>;

export class FakeTransport extends RomiCore.TransportEvents implements RomiCore.Transport {
  name: string = 'fake';

  constructor(doorStateFactory?: DoorStateFactory) {
    super();
    this._doorStateFactory = doorStateFactory || fakeDoorStates;
  }

  createPublisher<Message extends unknown>(
    topic: RomiCore.RomiTopic<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Publisher<Message> {
    return {
      publish: debug,
    };
  }

  subscribe<Message extends unknown>(
    topic: RomiCore.RomiTopic<Message>,
    cb: RomiCore.SubscriptionCb<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Subscription {
    debug('subscribe %s', topic.topic);
    let timer: number;
    switch (topic) {
      case RomiCore.doorStates: {
        const doorStates = this._doorStateFactory();
        timer = window.setInterval(() => {
          for (const state of Object.values(doorStates)) {
            debug('publishing door state, %s', state.door_name);
            cb(state as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.liftStates: {
        const liftStates = fakeLiftStates();
        timer = window.setInterval(() => {
          for (const state of Object.values(liftStates)) {
            debug('publishing lift state, %s', state.lift_name);
            cb(state as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.fleetStates: {
        const fleets = fakeFleets();
        timer = window.setInterval(() => {
          for (const fleet of fleets) {
            debug('publishing fleet state, %s', fleet.name);
            cb(fleet as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.dispenserStates: {
        const dispenserStates = fakeDispenserStates();
        timer = window.setInterval(() => {
          for (const state of Object.values(dispenserStates)) {
            debug('publishing dispenser state, %s', state.guid);
            cb(state as Message);
          }
        }, 1000);
        break;
      }

      case RomiCore.taskSummaries: {
        const taskSummaries = fakeTaskSummary();
        timer = window.setInterval(() => {
          for (const task of Object.values(taskSummaries)) {
            debug('publishing task, %s', task.task_id);
            cb(task as Message);
          }
        }, 1000);
        break;
      }
    }

    return {
      unsubscribe: () => {
        clearInterval(timer);
        debug('unsubscribed %s', topic.topic);
      },
    };
  }

  async call<Request extends unknown, Response extends unknown>(
    service: RomiCore.RomiService<Request, Response>,
    req: Request,
  ): Promise<Response> {
    if (service.service === 'get_building_map') {
      return new RomiCore.GetBuildingMap_Response(await buildingMap()) as any;
    }
    throw new Error('not implemented');
  }

  createService<Request extends unknown, Response extends unknown>(
    service: RomiCore.RomiService<Request, Response>,
  ): RomiCore.Service<Request, Response> {
    throw new Error('not implemented');
  }

  destroy(): void {}

  private _doorStateFactory: DoorStateFactory;
}

export default FakeTransport;
