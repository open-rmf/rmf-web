import React from 'react';

import { JsonForms } from '@jsonforms/react';
import { materialRenderers, materialCells } from '@jsonforms/material-renderers';
import { ConfirmationDialog, ConfirmationDialogProps } from './confirmation-dialog';

export interface JsonFormDialogProps extends Omit<ConfirmationDialogProps, 'onSubmit'> {
  /**
   * The JSON Schema for the form.
   */
  schema: any;
  /**
   * The UI Schema for the form.
   */
  uischema: any;
  /**
   * The initial data for the form.
   */
  initialData: any;
  /**
   * Callback fired when the form is submitted.
   * @param data The data from the form.
   */
  onSubmit: (data: any) => void;
}

export function JsonFormDialog({
  schema,
  uischema,
  initialData,
  onSubmit,
  ...otherProps
}: JsonFormDialogProps): JSX.Element {
  const [data, setData] = React.useState(initialData);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit(data);
  };

  return (
    <ConfirmationDialog onSubmit={handleSubmit} title="JSON Form Task" {...otherProps}>
      <JsonForms
        schema={schema}
        uischema={uischema}
        data={data}
        renderers={materialRenderers}
        cells={materialCells}
        onChange={({ data }) => setData(data)}
      />

      <p>JSON Forms will be rendered here. You need to install and uncomment the dependencies.</p>
      <p>Schema: {JSON.stringify(schema)}</p>
      <p>UI Schema: {JSON.stringify(uischema)}</p>
    </ConfirmationDialog>
  );
}
