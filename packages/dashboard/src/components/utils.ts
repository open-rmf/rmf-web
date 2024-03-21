import Ajv from 'ajv';
import schema from 'api-client/schema';
import { AxiosError } from 'axios';

export function getApiErrorMessage(error: unknown): string {
  const body = (error as AxiosError).response?.data as any;
  return typeof body === 'object' && body.detail ? body.detail : '';
}

export const ajv = new Ajv();

Object.entries(schema.components.schemas).forEach(([k, v]) => {
  ajv.addSchema(v, `#/components/schemas/${k}`);
});
