import * as winston from 'winston';

interface CustomLogger extends winston.Logger {
  child: (metadata: { label?: string }) => CustomLogger;
}
export type Logger = CustomLogger;

const labelToMessage = winston.format((info) => {
  if (info.label !== undefined) {
    info.message = `[${info.label}] ${info.message}`;
    info.label = undefined;
  }
  return info;
});

export const logger: CustomLogger = winston.createLogger({
  format: winston.format.combine(
    labelToMessage(),
    winston.format.simple(),
    winston.format.colorize({
      all: true,
    }),
  ),
  transports: new winston.transports.Console(),
});

export default logger;
