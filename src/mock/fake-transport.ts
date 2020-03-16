import * as RomiCore from '@osrf/romi-js-core-interfaces';
import buildingMap from './data/building-map';
import doorStates from './data/door-states';
import fleets from './data/fleets';
import liftStates from './data/lift-states';
import dispenserStates from './data/dispenser-states';

export class FakeTransport extends RomiCore.TransportEvents implements RomiCore.Transport {
  name: string = 'fake';

  createPublisher<Message extends object>(
    topic: RomiCore.RomiTopic<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Publisher<Message> {
    return {
      publish: console.log,
    };
  }

  subscribe<Message extends object>(
    topic: RomiCore.RomiTopic<Message>,
    cb: RomiCore.SubscriptionCb<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Subscription {
    console.log('subscribe:', topic);
    switch (topic) {
      case RomiCore.doorStates: {
        setInterval(() => {
          for (const state of Object.values(doorStates)) {
            cb(state as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.liftStates: {
        setInterval(() => {
          for (const state of Object.values(liftStates)) {
            cb(state as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.fleetStates: {
        setInterval(() => {
          for (const fleet of fleets) {
            cb(fleet as Message);
          }
        }, 1000);
        break;
      }
      case RomiCore.dispenserStates: {
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

  async call<Request extends object, Response extends object>(
    service: RomiCore.RomiService<Request, Response>,
    req: Request,
  ): Promise<Response> {
    if (service.service === 'get_building_map') {
      return new RomiCore.GetBuildingMap_Response(await buildingMap()) as any;
    }
    throw new Error('not implemented');
  }

  destroy(): void {}
}

export default FakeTransport;
