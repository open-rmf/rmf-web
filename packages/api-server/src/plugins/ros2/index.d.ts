import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { RomiService, RomiTopic } from '@osrf/romi-js-core-interfaces';
import { Argv } from 'yargs';
import ApiGateway, { Logger, Sender } from '../../api-gateway';
export declare function options(
  yargs: Argv,
): Argv<{
  ros2NodeName: string;
}>;
declare type ConfigType = ReturnType<typeof options>['argv'];
export declare type Ros2Topic<Message = unknown, T = Omit<RomiTopic<Message>, 'validate'>> = {
  -readonly [P in keyof T]: T[P];
};
export declare type Ros2Service<
  Request = unknown,
  Response = unknown,
  T = Omit<RomiService<Request, Response>, 'validateRequest' | 'validateResponse'>
> = {
  -readonly [P in keyof T]: T[P];
};
export interface SubscribeParams<T = unknown> {
  topic: Ros2Topic<T>;
}
export interface SubscribeResult {
  id: number;
}
export interface UnsubscribeParams {
  id: number;
}
export interface MessageResult<T = unknown> {
  message: T;
}
export interface CreatePublisherParams<T = unknown> {
  topic: Ros2Topic<T>;
}
export interface PublishParams<T = unknown> {
  id: number;
  message: T;
}
export interface DestroyPublisherParams {
  id: number;
}
export interface ServiceCallParams<T = unknown> {
  request: T;
  service: Ros2Service<T, unknown>;
}
export declare function onLoad(config: ConfigType, api: ApiGateway): Promise<void>;
export default class Ros2Plugin {
  transport: RomiCore.Transport;
  private _logger;
  get subscriptionCount(): number;
  get innerSubscriptionCount(): number;
  get publisherCount(): number;
  get innerPublisherCount(): number;
  constructor(transport: RomiCore.Transport, _logger: Logger);
  subscribe(params: SubscribeParams, sender: Sender<MessageResult>): SubscribeResult;
  unsubscribe(params: UnsubscribeParams): void;
  createPublisher(params: CreatePublisherParams, sender: Sender<never>): number;
  publish(params: PublishParams): void;
  serviceCall(params: ServiceCallParams): Promise<unknown>;
  toRomiTopic<Message = unknown>(ros2Topic: Ros2Topic<Message>): RomiTopic<Message>;
  toRomiService<Request = unknown, Response = unknown>(
    ros2Service: Ros2Service<Request, Response>,
  ): RomiService<Request, Response>;
  private _subscriptions;
  private _innerSubscriptions;
  private _publishers;
  private _innerPublishers;
  private _idCounter;
}
export {};
