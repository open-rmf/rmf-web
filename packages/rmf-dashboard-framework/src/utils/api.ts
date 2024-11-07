import { AxiosError } from 'axios';

export function getApiErrorMessage(error: unknown): string {
  const body = (error as AxiosError).response?.data as any;
  return typeof body === 'object' && body.detail ? body.detail : '';
}
