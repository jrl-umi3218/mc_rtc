function affixWidth() {
  var affix = $('#affix-overflow');
  var width = $('#content-column').width();
  affix.width(width);
}

function docson_widget(path)
{
  return "<script src=\"docson/widget.js\" data-schema=\"../schemas/" + path + "\"></script>";
}

function make_list_group(dir, objects)
{
  var html = "";
  for(i in objects)
  {
    var o = objects[i];
    var schema = dir + '/' + o + '.json';
    html += '<a href="#' + schema + '" class="list-group-item" data-schema="' + schema + '">' + o +'</a>\n';
  }
  return html;
}

function make_example_panel(id, data)
{
  id = id + '-example';
  id = id.replace(/#/g, "");
  id = id.replace(/\//g, "_");
  id = id.replace(/./g, "_");
  var html = "";
  html += '<div class="panel panel-default">';
  html += '<div class="panel-heading"><a data-toggle="collapse" href="#' + id + '">Example</a></div>';
  html += '<div id="' + id + '" class="panel-collapse panel-body collapse">';
  html += '<pre id="'+ id + '-data">' + data + '</pre>';
  html += '<button class="copy-button" data-clipboard-target="#' + id + '-data">'
  html += 'Copy to clipboard'
  html += '</button>'
  html += '</div></div>';
  return html;
}

$(document).ready(function()
{
  var eigen_objects = [
  'Matrix3d',
  'Matrix6d',
  'Matrix6Xd',
  'Quaterniond',
  'Vector2d',
  'Vector3d',
  'Vector6d',
  'VectorXd',
  ];
  $('#eigen-objects').html(make_list_group('eigen', eigen_objects));

  var sva_objects = [
  'ForceVecd',
  'MotionVecd',
  'PTransformd',
  'RBInertiad',
  ];
  $('#sva-objects').html(make_list_group('sva', sva_objects));

  var rbd_objects = [
  'Joint.Type',
  'Joint',
  'Body',
  'MultiBody',
  'MultiBodyConfig',
  ];
  $('#rbd-objects').html(make_list_group('rbd', rbd_objects));

  var tasks_objects = [
   'JointGains',
  ];
  $('#tasks-objects').html(make_list_group('tasks', tasks_objects));

  var mc_rbdyn_urdf_objects = [
  'Geometry.Box',
  'Geometry.Cylinder',
  'Geometry.Mesh',
  'Geometry.Sphere',
  'Geometry',
  'Visual',
  ];
  $('#mc_rbdyn_urdf-objects').html(make_list_group('mc_rbdyn_urdf', mc_rbdyn_urdf_objects));

  var mc_rbdyn_objects = [
  'Base',
  'BodySensor',
  'Collision',
  'Contact',
  'Flexibility',
  'ForceSensor',
  'Plane',
  'PolygonInterpolator',
  'RobotModule.Gripper',
  'RobotModule',
  'Springs',
  'SurfacePtr',
  'PlanarSurfacePtr',
  'CylindricalSurfacePtr',
  'GripperSurfacePtr',
  ];
  $('#mc_rbdyn-objects').html(make_list_group('mc_rbdyn', mc_rbdyn_objects));

  var constraintset_objects = [
  'BoundedSpeedConstr',
  'CollisionsConstraint',
  'CoMIncPlaneConstr',
  'ContactConstraint',
  'KinematicsConstraint',
  'DynamicsConstraint'
  ]
  $('#constraintset-objects').html(make_list_group('constraintset', constraintset_objects));

  var metatask_objects = [
  'AddContactTask',
  'AdmittanceTask',
  'BSplineTrajectoryTask',
  'ExactCubicTrajectoryTask',
  'ComplianceTask',
  'CoMTask',
  'CoPTask',
  'EndEffectorTask',
  'GazeTask',
  'LookAtTask',
  'LookAtSurfaceTask',
  'LookAtTFTask',
  'OrientationTask',
  'PBVSTask',
  'PositionTask',
  'PostureTask',
  'RelativeEndEffectorTask',
  'RemoveContactTask',
  'SurfaceTransformTask',
  'TrajectoryTask',
  'VectorOrientationTask'
  ]
  $('#metatask-objects').html(make_list_group('metatask', metatask_objects));

  $('.list-group-item').on('click', function() {
            var $this = $(this);
            var schema = $this.data('schema');
            if(schema != null)
            {
              $('a').removeClass('active');
              $this.addClass('active');
              var html = docson_widget(schema);
              var href = this.attributes["href"].value;
              $('#example').empty();
              $.get( 'examples/' + schema, function(data) {
                html += make_example_panel(href, data);
                $('#example').html(html);
              }, "text");
              $('#content').html(html);
            }
            $('.glyphicon', this)
              .toggleClass('glyphicon-chevron-right')
              .toggleClass('glyphicon-chevron-down');
        });

  var param = window.location.hash.substring(1);
  if(param.length)
  {
    $('#content').html(docson_widget(param));
    $('#example').empty();
    $.get( 'examples/' + param, function(data) {
      var html = make_example_panel(param, data);
      $('#example').html(html);
    }, "text");
    var links = document.getElementsByTagName('a');
    for(var i = 0; i < links.length; i++)
    {
      if(links.item(i).attributes["href"].value == window.location.hash)
      {
        var link = $(links.item(i));
        link.addClass('active');
        link.parent().addClass('in');
        $('.glyphicon', link.parent().prev())
          .toggleClass('glyphicon-chevron-right')
          .toggleClass('glyphicon-chevron-down');
      }
    }
  }

  affixWidth();

  new Clipboard('.copy-button');
});

$(window).resize(function() {
  affixWidth();
});
