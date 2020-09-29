import type {
  CreatePublisherParams,
  MessageResult,
  PublishParams,
  Ros2Service,
  Ros2Topic,
  ServiceCallParams,
  SubscribeParams,
  SubscribeResult,
  UnsubscribeParams,
} from '@osrf/romi-js-api-server/src/plugins/ros2';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { TransportEvents } from '@osrf/romi-js-core-interfaces';
import ApiClient from './api-client';

export default class Ros2Transport extends TransportEvents implements RomiCore.Transport {
  readonly name = 'romi_dashboard';
  constructor(public api: ApiClient) {
    super();
  }

  createPublisher<Message extends unknown>(
    topic: RomiCore.RomiTopic<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Publisher<Message> {
    const ros2Topic = this._toRos2Topic(topic, options);
    const params: CreatePublisherParams = {
      topic: ros2Topic,
    };
    let publisherId: number | undefined;
    this.api.rpcRequest<number>('ros2CreatePublisher', params, (result) => {
      publisherId = result;
    });

    return {
      publish: (msg: Message) => {
        if (publisherId === undefined) {
          console.warn('publisher not ready');
          return;
        }

        const params: PublishParams = {
          id: publisherId,
          message: msg,
        };
        this.api.rpcNotify('ros2Publish', params);
      },
    };
  }

  subscribe<Message extends unknown>(
    topic: RomiCore.RomiTopic<Message>,
    cb: RomiCore.SubscriptionCb<Message>,
    options?: RomiCore.Options | undefined,
  ): RomiCore.Subscription {
    const ros2Topic = this._toRos2Topic(topic, options);
    const callbackHandler = (result: MessageResult) => {
      cb(topic.validate(result.message));
    };
    let id: number | undefined = undefined;
    let handler: (result: any) => void = (result: SubscribeResult) => {
      id = result.id;
      handler = callbackHandler;
    };

    const params: SubscribeParams = {
      topic: ros2Topic,
    };
    this.api.rpcRequest<MessageResult | SubscribeResult>('ros2Subscribe', params, (result) => {
      handler(result);
    });

    const subscription: RomiCore.Subscription = {
      unsubscribe: () => {
        id !== undefined && this.api.rpcNotify<UnsubscribeParams>('ros2Unsubscribe', { id });
        this._subscriptions.delete(subscription);
      },
    };
    this._subscriptions.add(subscription);
    return subscription;
  }

  async call<Request extends unknown, Response extends unknown>(
    service: RomiCore.RomiService<Request, Response>,
    req: Request,
  ): Promise<Response> {
    const ros2Service = this._toRos2Service(service);
    const params: ServiceCallParams = {
      service: ros2Service,
      request: req,
    };
    return new Promise((res) =>
      this.api.rpcRequest<Response>('ros2ServiceCall', params, (result) => {
        res(service.validateResponse(result));
      }),
    );
  }

  createService<Request extends unknown, Response extends unknown>(
    service: RomiCore.RomiService<Request, Response>,
  ): RomiCore.Service<Request, Response> {
    throw new Error('Method not implemented.');
  }

  destroy(): void {
    this._subscriptions.forEach((subscription) => {
      subscription.unsubscribe();
    });
    this._subscriptions.clear();
  }

  private _subscriptions: Set<RomiCore.Subscription> = new Set();

  private _toRos2Topic(
    romiTopic: RomiCore.RomiTopic<unknown>,
    options?: RomiCore.Options | undefined,
  ): Ros2Topic<unknown> {
    const topic: RomiCore.RomiTopic<unknown> = {
      ...romiTopic,
      options: options || romiTopic.options,
    };
    delete (topic as any).validate;
    return topic;
  }

  private _toRos2Service(
    romiService: RomiCore.RomiService<unknown, unknown>,
  ): Ros2Service<unknown, unknown> {
    const service: RomiCore.RomiService<unknown, unknown> = {
      ...romiService,
    };
    delete (service as any).validateRequest;
    delete (service as any).validateResponse;
    return service;
  }
}
