#!/usr/bin/env node
import yargs from 'yargs';
import ApiGateway from './api-gateway';
export interface Plugin {
  options(yargs: yargs.Argv): void;
  onLoad(config: unknown, api: ApiGateway): Promise<void>;
}
