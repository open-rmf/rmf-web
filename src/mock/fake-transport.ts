import * as RomiCore from '@osrf/romi-js-core-interfaces';
import buildingMap from './data/building-map';

export class FakeTransport extends RomiCore.TransportEvents implements RomiCore.Transport {
  name: string = 'fake';

  constructor() {
    super();
  }

  createPublisher<Message extends object>(
    topic: RomiCore.RomiTopic<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Publisher<Message> {
    return {
      publish: () => {},
    };
  }

  subscribe<Message extends object>(
    topic: RomiCore.RomiTopic<Message>,
    cb: RomiCore.SubscriptionCb<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Subscription {
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
