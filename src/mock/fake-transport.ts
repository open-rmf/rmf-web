import * as RomiCore from '@osrf/romi-js-core-interfaces';
import buildingMap from './data/building-map';
import fleets from './data/fleets';

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
    if (topic === RomiCore.fleetStates) {
      setInterval(() => {
        for (const fleet of fleets) {
          cb(fleet as Message);
        }
      }, 1000);
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
      return new RomiCore.GetBuildingMap_Response(buildingMap) as any;
    }
    throw new Error('not implemented');
  }

  destroy(): void {}
}
