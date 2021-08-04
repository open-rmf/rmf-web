import { AxiosError } from 'axios';

export function getApiErrorMessage(error: AxiosError): string {
  return error.response?.data.detail || '';
}
