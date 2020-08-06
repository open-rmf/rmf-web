import React from 'react';

import AutoCompleteComponent from './BaseComponents/autocomplete';
import TextFieldComponent from './BaseComponents/textfield';
import FormButtonComponent from './BaseComponents/form-button';

export default {
  title: 'Forms',
};

const States: boolean[] = [false, true];
const Messages: string[] = ['', 'This is an error'];
const labels: string[] = ['Correct Inputs', 'Wrong Inputs'];

export const AutoComplete = () => (
  <AutoCompleteComponent errorState={States} errorMessage={Messages} labels={labels} />
);

export const TextField = () => (
  <TextFieldComponent errorState={States} errorMessage={Messages} labels={labels} />
);

export const FormButton = () => <FormButtonComponent />;
