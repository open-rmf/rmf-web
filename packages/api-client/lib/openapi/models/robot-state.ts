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
 * @interface RobotState
 */
export interface RobotState {
  /**
   *
   * @type {string}
   * @memberof RobotState
   */
  name?: any;
  /**
   *
   * @type {string}
   * @memberof RobotState
   */
  model?: any;
  /**
   *
   * @type {string}
   * @memberof RobotState
   */
  task_id?: any;
  /**
   *
   * @type {number}
   * @memberof RobotState
   */
  seq?: any;
  /**
   *
   * @type {RobotMode}
   * @memberof RobotState
   */
  mode?: any;
  /**
   *
   * @type {number}
   * @memberof RobotState
   */
  battery_percent?: any;
  /**
   *
   * @type {Location}
   * @memberof RobotState
   */
  location?: any;
  /**
   *
   * @type {Array&lt;Location&gt;}
   * @memberof RobotState
   */
  path?: any;
}
