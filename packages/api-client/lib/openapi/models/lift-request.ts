/* tslint:disable */
/* eslint-disable */
/**
 * RMF API Server
 * No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)
 *
 * OpenAPI spec version: 0.1.0
 *
 *
 * NOTE: This class is auto generated by the swagger code generator program.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */
/**
 *
 * @export
 * @interface LiftRequest
 */
export interface LiftRequest {
  /**
   * https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg
   * @type {number}
   * @memberof LiftRequest
   */
  request_type: any;
  /**
   * https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_lift_msgs/msg/LiftRequest.msg
   * @type {number}
   * @memberof LiftRequest
   */
  door_mode: any;
  /**
   *
   * @type {string}
   * @memberof LiftRequest
   */
  destination: any;
}
