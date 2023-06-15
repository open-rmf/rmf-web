/**
 * Health status as an enum as the api-client does not generate and expose
 * HealthStatus.
 * TODO(AA): Generate unexposed enums for api-client to reduce duplicated
 * declarations.
 */
export enum HealthStatus {
  Healthy = 'Healthy',
  Unhealthy = 'Unhealthy',
  Dead = 'Dead',
}

/**
 * Returns the Op Mode based on the health status.
 * @param healthStatus The enum strings from HealthStatus
 * @returns Op mode
 */
export function healthStatusToOpMode(healthStatus: string): string {
  switch (healthStatus) {
    case HealthStatus.Healthy:
      return 'ONLINE';
    case HealthStatus.Unhealthy:
      return 'UNSTABLE';
    case HealthStatus.Dead:
      return 'OFFLINE';
    default:
      return `Unknown ${healthStatus}`;
  }
}
