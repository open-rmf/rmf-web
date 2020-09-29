import React from 'react';

import AutoCompleteComponent from './baseComponents/autocomplete';
import TextFieldComponent from './baseComponents/textfield';
import FormButtonComponent from './baseComponents/form-button';

export default {
  title: 'Forms',
};

const states: boolean[] = [false, true];
const messages: string[] = ['', 'This is an error'];
const labels: string[] = ['Correct Inputs', 'Wrong Inputs'];

export const autoComplete = () => (
  <AutoCompleteComponent errorState={states} errorMessage={messages} labels={labels} />
);

export const textField = () => (
  <TextFieldComponent errorState={states} errorMessage={messages} labels={labels} />
);

export const formButton = () => <FormButtonComponent />;
