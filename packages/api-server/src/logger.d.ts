import * as winston from 'winston';
export interface CustomLogger extends winston.Logger {
  child: (metadata: { tag: unknown }) => CustomLogger;
}
export declare const logger: CustomLogger;
export default logger;
