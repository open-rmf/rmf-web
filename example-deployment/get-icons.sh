
cd ws/rmf-web/packages/dashboard
echo -e '{"repoUrl":"https://github.com/open-rmf/rmf_demos.git","folder":"rmf_demos_dashboard_resources/office/","branch":"main"}' > .resources.json
node ./scripts/setup/get-icons.js
cd -