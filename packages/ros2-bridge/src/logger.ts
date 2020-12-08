import * as winston from 'winston';

interface CustomLogger extends winston.Logger {
  child: (metadata: { tag: unknown }) => CustomLogger;
}
export type Logger = CustomLogger;

export const logger: CustomLogger = winston.createLogger({
  format: winston.format.combine(
    winston.format.printf((info) => {
      const tag = info.tag ? `[${info.tag}]` : '';
      return `${info.level.toUpperCase()}: ${tag} ${info.message}`;
    }),
    winston.format.colorize({
      all: true,
    }),
  ),
  transports: new winston.transports.Console(),
});

export default logger;
