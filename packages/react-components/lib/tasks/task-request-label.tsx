import schema from 'api-client/dist/schema';

export interface TaskRequestLabel {
  description: {
    task_name?: string;
    unix_millis_warn_time?: number;
    pickup?: string;
    destination?: string;
    cart_id?: string;
  };
}

// const errIdx = obj.findIndex((req) => !ajv.validate(schema.components.schemas.TaskRequest, req));
