import Ajv from 'ajv';
import schema from 'api-client/schema';
import { AxiosError, isAxiosError } from 'axios';

export function getApiErrorMessage(error: unknown): string {
  if (!isAxiosError(error)) {
    return '';
  }

  const response = (error as AxiosError<string>).response;
  if (!response) {
    return '';
  }
  return `${response.data}`;
}

export const ajv = new Ajv();

Object.entries(schema.components.schemas).forEach(([k, v]) => {
  ajv.addSchema(v, `#/components/schemas/${k}`);
});
