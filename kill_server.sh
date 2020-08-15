server_process_id=$(netstat -tulpn | grep 65432 | awk -F ' ' '{print $7}' | cut -d'/' -f1)
sudo kill $server_process_id