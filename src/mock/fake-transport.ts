import * as RomiCore from '@osrf/romi-js-core-interfaces';
import debug from 'debug';
import buildingMap from './data/building-map';
import fakeDispenserStates from './data/dispenser-states';
import fakeDoorStates from './data/door-states';
import fakeFleets from './data/fleets';
import fakeLiftStates from './data/lift-states';

export class FakeTransport extends RomiCore.TransportEvents implements RomiCore.Transport {
  name: string = 'fake';

  createPublisher<Message extends unknown>(
    topic: RomiCore.RomiTopic<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Publisher<Message> {
    return {
      publish: debug.log,
    };
  }

  subscribe<Message extends unknown>(
    topic: RomiCore.RomiTopic<Message>,
    cb: RomiCore.SubscriptionCb<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Subscription {
    debug.log('subscribe:', topic);
    switch (topic) {
      case RomiCore.doorStates: {
        const doorStates = fakeDoorStates();
        setInterval(() => {
          for (const state of Object.values(doorStates)) {
            cb(state as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.liftStates: {
        const liftStates = fakeLiftStates();
        setInterval(() => {
          for (const state of Object.values(liftStates)) {
            cb(state as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.fleetStates: {
        const fleets = fakeFleets();
        setInterval(() => {
          for (const fleet of fleets) {
            cb(fleet as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.dispenserStates: {
        const dispenserStates = fakeDispenserStates();
        setInterval(() => {
          for (const state of Object.values(dispenserStates)) {
            cb(state as Message);
          }
        }, 1000);
        break;
      }
    }
    return {
      unsubscribe: () => {},
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
}

export default FakeTransport;
