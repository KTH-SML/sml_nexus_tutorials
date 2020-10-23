function ErrorNexus2XCallback(~, message)
  global error_nexus2_x;
  error_nexus2_x(end+1) = message.Data;
end

