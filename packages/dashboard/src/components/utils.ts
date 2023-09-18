import Ajv from 'ajv';
import schema from 'api-client/schema';
import { AxiosError } from 'axios';

export function getApiErrorMessage(error: unknown): string {
  return (error as AxiosError).response?.data.detail || '';
}

export const ajv = new Ajv();

Object.entries(schema.components.schemas).forEach(([k, v]) => {
  ajv.addSchema(v, `#/components/schemas/${k}`);
});
